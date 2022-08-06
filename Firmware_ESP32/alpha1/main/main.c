#include <stdio.h>
#include "esp_log.h"

#include "adc_st.h"
#include "bt_st.h"
#include "dac_st.h"
#include "i2c_st.h"
#include "ipu_st.h"
#include "ism330_st.h"
#include "rtc_st.h"
#include "kalman_st.h"

#include "esp_system.h"
#include "esp_console.h"
#include "esp_check.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "driver/uart.h"
#include "driver/ledc.h"

// Define useful constants.
#define PI          (3.14159265359)
#define PI2         (6.28318530718)
#define PI180       (0.01745329251)
#define PI180I      (57.2957795131)

// Used for the Kalman filter.
#define RESTRICT_PITCH

/*
 * Notes:
 *
 * Data frame:
 * 2 bytes = time
 * 2 bytes = pitch
 * 2 bytes = roll
 * 2 bytes = torque data
 * 
 * 8 bytes = total
 * 
 * Pre-allocated memory to store before a test:
 * 8 bytes, assume 500 updates per second.
 * = 4,000 bytes per second
 * Assuming 60 second test, we will need to pre-allocate 240,000 bytes of RAM.
 * 
 */

// Initialize the bluetooth module.
struct bt_config_t bt = 
{
    .device_name = "STRASAnkle Gamma"
};

struct kalman_filter fkp;
struct kalman_filter fkr;
struct kalman_filter skp;
struct kalman_filter skr;

// Begin statFile parameters. *********************************

// Get the current battery status.
uint8_t batteryPercentage = 100;

// Is the device currently charging?
bool charging;

// Is the shin strap connected? TODO
bool shinStrapConnected;

// Is the shin strap malfunctioning?
bool shinMalf;

// Is the device currently in a test?
bool testing;

// Store the heel length for the torque calculations.
uint8_t heelLength;

// Store the calibration status.
bool calibrateStatus = false;

// Store if the variable states have changed and we need to
// send it back to the computer.
bool statFileStateChanged = false;

// End statFile parameters. ***********************************

// ADC instance.
spi_device_handle_t adc;

// Detect button edge press.
bool btn1Edge = false;
bool btn2Edge = false;

// Store button states.
uint8_t btn1State = 0;
uint8_t btn2State = 0;

// Detect if the bluetooth was connected for the first time in the main loop.
bool firstConnection = true;

// First pass of sending the testFile? (Send notification & initial packet).
bool testingFirst = false;
bool testingFinished = false;
uint8_t *testFileData = NULL;
int32_t testFileIter = 0;
int32_t testFileUpTo = 0;

// Is the USB cable connected?
bool chargerConnected = false;

// Is the battery currently being charged? This variable determines the LED states.
bool currentlyCharging = false;

bool jointAngleCalculatorFirstRun = true;

double f_gyroXangle, f_gyroYangle; // Angle calculate using the gyro only
double f_compAngleX, f_compAngleY; // Calculated angle using a complementary filter
double f_kalAngleX, f_kalAngleY;   // Calculated angle using a Kalman filter
double s_gyroXangle, s_gyroYangle; // Angle calculate using the gyro only
double s_compAngleX, s_compAngleY; // Calculated angle using a complementary filter
double s_kalAngleX, s_kalAngleY;   // Calculated angle using a Kalman filter

// TODO: Attempt to offset the 180 degree issue.
double pitchOffset = 0.0, rollOffset = 0.0;

// Calibration offsets for the ADC.
double ch0_offset;
double ch1_offset;

// Iterator for pulserLUT.
uint8_t pulserIter = 0;

// Controlled by RedLED
bool greenLEDAlternator = false;

// Virtual look-up table which determines the green LED pulsing behaviour.
uint16_t pulserLUT(uint8_t i)
{
    return 100 * sin(PI * (double)i / 256.0) * sin(PI * (double)i / 256.0);
}

void led_set_duty(uint8_t percent)
{
    uint16_t duty = percent * 82;
    if (duty > 8191) duty = 8191;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// Send the miscFile to the GUI.
int8_t send_statFile(void)
{
    if (bt.connected == false) return -1;

    uint8_t case1[11];

    while (bt.write_available == false)
    {
        if (bt.connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    strcpy((char *)case1, "statFile,w");
    printf("sending \"statFile,w\".\n");
    if (bt_notify(case1, 10) != 0) return -1;

    while (bt.write_available == false)
    {
        if (bt.connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    strcpy((char *)case1, "statFile");
    printf("sending \"statFile\".\n");
    if (bt_write(case1, 8) != 0) return -1;

    case1[0] = batteryPercentage;
    case1[1] = 0;
    case1[2] = shinStrapConnected;
    case1[3] = 1;
    case1[4] = testingFirst;
    case1[5] = 0;
    case1[6] = (uint8_t)calibrateStatus;
    while (bt.write_available == false)
    {
        if (bt.connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    if (bt_write(case1, 7) != 0) return -1;

    strcpy((char *)case1, "EOF");
    while (bt.write_available == false)
    {
        if (bt.connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    if (bt_write(case1, 3) != 0) return -1;

    return 0;
}

void jointAngleCalculator(float f_acc_x, float f_acc_y, float f_acc_z,
                         float f_gyr_x, float f_gyr_y, float f_gyr_z,
                         float s_acc_x, float s_acc_y, float s_acc_z,
                         float s_gyr_x, float s_gyr_y, float s_gyr_z,
                         double dt,
                         float *r, float *p)
{
#ifdef RESTRICT_PITCH
    double f_pitch = atan(-f_acc_x / sqrt(f_acc_y * f_acc_y + f_acc_z * f_acc_z)) * PI180I;
    double f_roll = atan2(f_acc_y, f_acc_z) * PI180I;  
    double s_pitch = atan(-s_acc_x / sqrt(s_acc_y * s_acc_y + s_acc_z * s_acc_z)) * PI180I;
    double s_roll = atan2(s_acc_y, s_acc_z) * PI180I;
#else
    double f_roll = atan(f_acc_y / sqrt(f_acc_x * f_acc_x + f_acc_z * f_acc_z)) * PI180I;
    double f_pitch = atan2(-f_acc_x, f_acc_z) * PI180I;  
    double s_roll = atan(s_acc_y / sqrt(s_acc_x * s_acc_x + s_acc_z * s_acc_z)) * PI180I;
    double s_pitch = atan2(-s_acc_x, s_acc_z) * PI180I;
#endif

    double f_gyroXrate = (double)f_gyr_x; // deg/s
    double f_gyroYrate = (double)f_gyr_y; // deg/s
    double s_gyroXrate = (double)s_gyr_x; // deg/s
    double s_gyroYrate = (double)s_gyr_y; // deg/s

    if (jointAngleCalculatorFirstRun == true) 
    {
        // Set starting angle
        kalman_setAngle(&fkr, f_roll);
        kalman_setAngle(&fkp, f_pitch);
        kalman_setAngle(&skr, s_roll);
        kalman_setAngle(&skp, s_pitch);

        f_gyroXangle = f_roll;
        f_gyroYangle = f_pitch;
        f_compAngleX = f_roll;
        f_compAngleY = f_pitch;

        s_gyroXangle = s_roll;
        s_gyroYangle = s_pitch;
        s_compAngleX = s_roll;
        s_compAngleY = s_pitch;

        
    }

#ifdef RESTRICT_PITCH
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((f_roll < -90.0 && f_kalAngleX > 90.0) || (f_roll > 90.0 && f_kalAngleX < -90.0))
    {
        kalman_setAngle(&fkr, f_roll);
        f_compAngleX = f_roll;
        f_kalAngleX = f_roll;
        f_gyroXangle = f_roll;
    }
    else
        f_kalAngleX = kalman_getAngle(&fkr, f_roll, f_gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(f_kalAngleX) > 90.0)
        f_gyroYrate = -f_gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    f_kalAngleY = kalman_getAngle(&fkp, f_pitch, f_gyroYrate, dt); // Calculate the angle using a Kalman filter

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((s_roll < -90.0 && s_kalAngleX > 90.0) || (s_roll > 90.0 && s_kalAngleX < -90.0))
    {
        kalman_setAngle(&skr, s_roll);
        s_compAngleX = s_roll;
        s_kalAngleX = s_roll;
        s_gyroXangle = s_roll;
    }
    else
        s_kalAngleX = kalman_getAngle(&skr, s_roll, s_gyroXrate, dt); // Calculate the angle using a Kalman filter

    if (abs(s_kalAngleX) > 90.0)
        s_gyroYrate = -s_gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
    s_kalAngleY = kalman_getAngle(&skp, s_pitch, s_gyroYrate, dt); // Calculate the angle using a Kalman filter
#else
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((f_pitch < -90.0 && f_kalAngleY > 90.0) || (f_pitch > 90.0 && f_kalAngleY < -90.0))
    {
        kalman_setAngle(&fkp, f_pitch);
        f_compAngleY = f_pitch;
        f_kalAngleY = f_pitch;
        f_gyroYangle = f_pitch;
    }
    else
        f_kalAngleY = kalman_getAngle(&fkp, f_pitch, f_gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(f_kalAngleY) > 90.0)
        f_gyroXrate = -f_gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    f_kalAngleX = kalman_getAngle(&fkr, f_roll, f_gyroXrate, dt); // Calculate the angle using a Kalman filter

    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((s_pitch < -90.0 && s_kalAngleY > 90.0) || (s_pitch > 90.0 && s_kalAngleY < -90.0))
    {
        kalman_setAngle(&skp, s_pitch);
        s_compAngleY = s_pitch;
        s_kalAngleY = s_pitch;
        s_gyroYangle = s_pitch;
    }
    else
        s_kalAngleY = kalman_getAngle(&skp, s_pitch, s_gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(s_kalAngleY) > 90.0)
        s_gyroXrate = -s_gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    s_kalAngleX = kalman_getAngle(&skr, s_roll, s_gyroXrate, dt); // Calculate the angle using a Kalman filter

#endif


    f_gyroXangle += f_gyroXrate * dt; // Calculate gyro angle without any filter
    f_gyroYangle += f_gyroYrate * dt;
    //f_gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //f_gyroYangle += kalmanY.getRate() * dt;

    s_gyroXangle += s_gyroXrate * dt; // Calculate gyro angle without any filter
    s_gyroYangle += s_gyroYrate * dt;
    //f_gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //f_gyroYangle += kalmanY.getRate() * dt;


    f_compAngleX = 0.98 * (f_compAngleX + f_gyroXrate * dt) + 0.02 * f_roll; // Calculate the angle using a Complimentary filter
    f_compAngleY = 0.98 * (f_compAngleY + f_gyroYrate * dt) + 0.02 * f_pitch;

    s_compAngleX = 0.98 * (s_compAngleX + s_gyroXrate * dt) + 0.02 * s_roll; // Calculate the angle using a Complimentary filter
    s_compAngleY = 0.98 * (s_compAngleY + s_gyroYrate * dt) + 0.02 * s_pitch;



    // Reset the gyro angle when it has drifted too much
    if (f_gyroXangle < -180.0 || f_gyroXangle > 180.0)
        f_gyroXangle = f_kalAngleX;
    if (f_gyroYangle < -180.0 || f_gyroYangle > 180.0)
        f_gyroYangle = f_kalAngleY;

    if (s_gyroXangle < -180.0 || s_gyroXangle > 180.0)
        s_gyroXangle = s_kalAngleX;
    if (s_gyroYangle < -180.0 || s_gyroYangle > 180.0)
        s_gyroYangle = s_kalAngleY;

    static double prev_r;
    static double prev_p;

    if (jointAngleCalculatorFirstRun == true)
    {
        jointAngleCalculatorFirstRun = false;
        prev_r = *r;
        prev_p = *p;
    }

    // Joint angle calculation.
    //*r = s_kalAngleX - f_kalAngleX;
    //*p = s_kalAngleY - f_kalAngleY;

    *r = (prev_r * 0.7) + (0.3 * (s_kalAngleX - f_kalAngleX));
    *p = (prev_p * 0.7) + (0.3 * (s_kalAngleY - f_kalAngleY));

    prev_r = *r;
    prev_p = *p;
}

void testFileBTSender(void * pvParameters)
{
    printf("testFileBTSender task started.\n");

    // Initialize variables for sending text.
    uint8_t case1[11];

    while (1)
    {
        // Give time to other tasks.
        vTaskDelay(1000 / portTICK_RATE_MS);

        // Check if the device is connected. If not, don't bother sending the test.
        if (testingFinished == false || bt.connected == false) continue;

        // Notify the GUI.
        while (bt.write_available == false) vTaskDelay(10);
        strcpy((char *)case1, "testFile,w");
        printf("sending \"testFile,w\".\n");
        bt_notify(case1, 10);

        // Initial packet.
        while (bt.write_available == false) vTaskDelay(10);
        strcpy((char *)case1, "testFile");
        printf("sending \"testFile\".\n");
        bt_write(case1, 8);

        while (1)
        {
            while (bt.write_available == false) vTaskDelay(10);

            // Is there more data to send?
            int32_t diff = testFileUpTo - testFileIter;
            if (diff > 512) diff = 512;

            // If testing is still going on, wait for max packet size before sending.
            if (testingFinished == false && diff < 512) continue;

            // Remove 8 LSB (effectively round down to nearest 8).
            diff &= 0xFFFFFFF8;

            // Debug test file iterator as we move through the file.
            printf("testFileIter = %d.\ntestFileUpTo = %d.\ndiff = %d.\n", testFileIter, testFileUpTo, diff);

            // Only send data if there is data available to send.
            if (diff > 0)
            {
                // Send the data the other task just read.
                bt_write(&testFileData[testFileIter], diff);

                // Increment this task's file pointer indicating we are past where the data collection task has reached.
                testFileIter += diff;
            }
            else
            {
                strcpy((char *)case1, "EOF");
                while (bt.write_available == false) vTaskDelay(10 / portTICK_RATE_MS);
                bt_write((uint8_t *)case1, 3);
                testing = false;
                testingFinished = false;
                printf("finished sending testFile.\n");
                // Don't free memory, use for next test.
                //if (testFileData)
                //    free(testFileData);
                
                break;
            }
        }
    }
}

void testFileCollector(void * pvParameters)
{
    printf("testFileCollector task started.\n");
    // Initial RTOS time in ms.
    int dt = xTaskGetTickCount() * portTICK_RATE_MS;
    int prevTime = xTaskGetTickCount() * portTICK_RATE_MS;
    int16_t t = 0;

    // First pass, allocate memory and send required stuff.
    // Allocate test file memory.
    printf("pre-malloc.\n");
    testFileData = malloc(400000);
    if (testFileData)
    {
        printf("malloc ran successfully.\n");
    }
    else
    {
        printf("malloc fail!\n");
    }

    while (1)
    {
        // Initialize variables to store the shin strap data.
        uint8_t data[12];

        // Initialize variables to store the load cell data.
        double ch0v, ch1v;
        double ch0vacc = 0.0, ch1vacc = 0.0;
        
        // If not actively sending test file.
        if (testing == false || testingFinished == true)
        {
            // Give time to other tasks to do other things.
            vTaskDelay(100 / portTICK_RATE_MS);
            continue;
        }

        // Stop automatically, or if testingFinished was set by another task.
        if (testFileUpTo > 10000 || testingFinished == true)
        {
            testingFinished = true;
            printf("no longer collecting data.\n");
            vTaskDelay(1000 / portTICK_RATE_MS);
            continue;
        }

        // Read the shin strap data.
        for (int i = 0; i < 12; i++)
        {
            // Collect shin strap data.
            i2c_register_read_byte(IPU_I2C_ADDR, i, &data[i]);

            // At the same time, collect load cell data.
            adc_read_voltage(&adc, &ch0v, &ch1v);
            ch0vacc += ch0v;
            ch1vacc += ch1v;
        }

        // Read the button states from the registers.
        i2c_register_read_byte(IPU_I2C_ADDR, 0x0D, &btn1State);
        i2c_register_read_byte(IPU_I2C_ADDR, 0x0E, &btn2State);

        if (btn1State == true || btn2State == true)
        {
            testingFinished = true;
            continue;
        }

        // Subtract the offsets from the calibration procedure.
        ch0vacc -= ch0_offset * 12.0;
        ch1vacc -= ch1_offset * 12.0;

        // Negate the channel value since applied force
        // makes the ADC reading go down.
        ch0vacc = -ch0vacc;
        ch1vacc = -ch1vacc;

        // Torque calculation.
        uint16_t ch0_scaled = ch0vacc * (0.8202 - (heelLength / 0.00328)) * 10000.0;
        uint16_t ch1_scaled = ch1vacc * (0.4724 - (heelLength / 0.00328)) * 10000.0;

        uint16_t torque_data = ch0_scaled + ch1_scaled;

        // Get the current time from the chip.
        dt = (xTaskGetTickCount() * portTICK_RATE_MS) - prevTime;
        t += dt;
        prevTime = xTaskGetTickCount() * portTICK_RATE_MS;

        // Read the ISM330 data.
        float sgyrx = ism330_convert_gyr_x_dps((data[1] << 8) | data[0]);
        float sgyry = ism330_convert_gyr_y_dps((data[3] << 8) | data[2]);
        float sgyrz = ism330_convert_gyr_z_dps((data[5] << 8) | data[4]);
        float saccx = ism330_convert_acc_x_g((data[7] << 8) | data[6]);
        float saccy = ism330_convert_acc_y_g((data[9] << 8) | data[8]);
        float saccz = ism330_convert_acc_z_g((data[11] << 8) | data[10]);

        float faccx = ism330_get_acc_x_g();
        float faccy = ism330_get_acc_y_g();
        float faccz = ism330_get_acc_z_g();

        float fgyrx = ism330_get_gyr_x_dps();
        float fgyry = ism330_get_gyr_y_dps();
        float fgyrz = ism330_get_gyr_z_dps();

        float roll = 0.0;
        float pitch = 0.0;

        jointAngleCalculator(faccx, faccy, faccz, fgyrx, fgyry, fgyrz, saccx, saccy, saccz, sgyrx, sgyry, sgyrz, (double)dt, &roll, &pitch);

        int16_t roll2 = (int16_t)(roll * 364.0);
        int16_t pitch2 = (int16_t)(pitch * 364.0);

        // Update the testFileData and increment the upTo indexer for the other task.

        // Send current time.
        testFileData[testFileUpTo++] = (t >> 8) & 0xff;
        testFileData[testFileUpTo++] = t & 0xff;

        // Send pitch data with scale factor.
        testFileData[testFileUpTo++] = (pitch2 >> 8) & 0xff;
        testFileData[testFileUpTo++] = pitch2 & 0xff;

        // Send roll data with scale factor.
        testFileData[testFileUpTo++] = (roll2 >> 8) & 0xff;
        testFileData[testFileUpTo++] = roll2 & 0xff;

        // Send load cell data with scale factor. TODO
        testFileData[testFileUpTo++] = (torque_data >> 8) & 0xff;//(torque_data >> 8) & 0xff;
        testFileData[testFileUpTo++] = torque_data & 0xff;//torque_data & 0xff;
    }
}

void calibrateLoadCells(void)
{
    double ch0v = 0.0, ch1v = 0.0;
    double cal0 = 0.0, cal1 = 0.0;
    double sar = 1.0;

    // Define where we want the reading to settle (volts).
    const double mid = 2.5;

    // Calibrate the channels using successive approximation.
    while (sar > 0.00001)
    {
        // Set the new calibration values.
        dac_write(cal0, cal1);

        // Wait some time for the reading to settle.
        vTaskDelay(100 / portTICK_RATE_MS);

        // Read the new ADC channel values.
        double ch0vacc = 0.0;
        double ch1vacc = 0.0;

        for (uint8_t i = 0; i < 20; i++)
        {
            double ch0tmp;
            double ch1tmp;
            adc_read_voltage(&adc, &ch0tmp, &ch1tmp);
            ch0vacc += ch0tmp;
            ch1vacc += ch1tmp;
            vTaskDelay(1);
        }

        ch0v = ch0vacc / 20.0;
        ch1v = ch1vacc / 20.0;

        if (ch0v > mid) cal1 -= sar;
        else            cal1 += sar;
        if (ch1v > mid) cal0 -= sar;
        else            cal0 += sar;
        sar /= 2.0;
    }

    printf("final calibration values: %.4f, %.4f\n", cal0, cal1);
    
    ch0_offset = ch0v;
    ch1_offset = ch1v;

    printf("final adc offset values: %.4f, %.4f\n", ch0_offset, ch1_offset);
}


void BlueLED(void)
{
    printf("BlueLED task started.\n");

    // Set LED1 as output.
    gpio_config_t LED1 = 
    {
        .pin_bit_mask = GPIO_NUM_38,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&LED1);
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);
    while (1)
    {
        if (bt.initialized == true && chargerConnected == false)
        {
            // Charger is not connected.
            if (bt.connected == true)
            {
                gpio_set_level(GPIO_NUM_38, 1);
                vTaskDelay(1000 / portTICK_RATE_MS);
            }
            else
            {
                gpio_set_level(GPIO_NUM_38, 1);
                vTaskDelay(250 / portTICK_RATE_MS);
                gpio_set_level(GPIO_NUM_38, 0);
                vTaskDelay(250 / portTICK_RATE_MS);
            }
        }
        else
        {
            // Charger is connected.
            gpio_set_level(GPIO_NUM_38, (uint8_t)bt.connected);
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
}

void RedLED(void)
{
    printf("RedLED task started.\n");

    // Set LED2 as output.
    gpio_config_t LED2 = 
    {
        .pin_bit_mask = GPIO_NUM_45,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&LED2);
    gpio_set_direction(GPIO_NUM_45, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_45, 1);

    while (1)
    {
        if (calibrateStatus == false)
        {
            if (shinStrapConnected == true)
            {
                // Shin strap connected, solid red LED.
                gpio_set_level(GPIO_NUM_45, 1);
                vTaskDelay(250 / portTICK_RATE_MS);
            }
            else
            {
                // Shin strap not connected, blinking red LED.
                gpio_set_level(GPIO_NUM_45, 1);
                vTaskDelay(250 / portTICK_RATE_MS);
                gpio_set_level(GPIO_NUM_45, 0);
                vTaskDelay(250 / portTICK_RATE_MS);
            }
        }
        else
        {
            // Calibration procedure, alternate the red and green LEDs.
            gpio_set_level(GPIO_NUM_45, 1);
            greenLEDAlternator = false;
            vTaskDelay(250 / portTICK_RATE_MS);
            gpio_set_level(GPIO_NUM_45, 0);
            greenLEDAlternator = true;
            vTaskDelay(250 / portTICK_RATE_MS);
        }
    }
}

void GreenLED(void)
{
    printf("GreenLED task started.\n");

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = LEDC_TIMER_13_BIT,
        .freq_hz          = 5000,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_LOW_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = 46,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));

    while (1)
    {
        if (testing == true)
        {
            // Test has started, blinking green LED.
            led_set_duty(100);
            vTaskDelay(250 / portTICK_RATE_MS);
            led_set_duty(0);
            vTaskDelay(250 / portTICK_RATE_MS);
        }
        else if (calibrateStatus == true)
        {
            // Calibration procedure, alternate the red and green LEDs.
            led_set_duty(greenLEDAlternator * 100);
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        else
        {
            if (currentlyCharging == true)
            {
                // The battery is currently being charged.
                led_set_duty(pulserLUT(pulserIter++));
                vTaskDelay(10 / portTICK_RATE_MS);
            }
            else if(currentlyCharging == false && chargerConnected == true)
            {
                // The charger is connected and the battery is not being charged,
                // which means the battery is fully charged.
                led_set_duty(100);
                vTaskDelay(250 / portTICK_RATE_MS);
            }
            else
            {
                // The charger is not connected. Turn off the green LED.
                led_set_duty(0);
                vTaskDelay(250 / portTICK_RATE_MS);
            }
        }
    }
}

void gpio_init(void)
{
    // Set 5VA as output.
    gpio_config_t nVDD_5VA_EN = 
    {
        .pin_bit_mask = GPIO_NUM_35,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&nVDD_5VA_EN);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_35, 1);

    // Set nCHG_OK as input.
    gpio_config_t nCHG_OK = 
    {
        .pin_bit_mask = GPIO_NUM_39,
        .mode = GPIO_MODE_INPUT,
    };
    gpio_config(&nCHG_OK);
    gpio_set_direction(GPIO_NUM_39, GPIO_MODE_INPUT);

    // Set nCHG_EN as output.
    gpio_config_t nCHG_EN = 
    {
        .pin_bit_mask = GPIO_NUM_17,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&nCHG_EN);
    gpio_set_direction(GPIO_NUM_17, GPIO_MODE_OUTPUT);
}

void app_main(void)
{
    gpio_init();

    // Initialize the sensors.
    i2c_master_init(36, 37, 400000);

    // Initialize the accelerometer and gyroscope on the foot plate.
    ism330_init();

    // Initialize the bluetooth module.
    bt_init(&bt);

    // Initialize ADC.
    adc_init(&adc, 7);

    // Initialize DAC. Use a high sampling rate for now minimize avoid noise on the output (how to avoid this?).
    dac_init(96000);

    // Initialize the Kalman filters.
    kalman_init(&fkp);
    kalman_init(&fkr);
    kalman_init(&skp);
    kalman_init(&skr);

    // Create task for sending data to the computer (pinned to core 0).
    TaskHandle_t xHandle1 = NULL;
    static uint8_t ucParameterToPass = 0;
    xTaskCreate(testFileBTSender, "TESTFILE_SNDR", 8192, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle1);
    // xTaskCreatePinnedToCore(testFileBTSender, "TESTFILE_SNDR", 8192, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle1, 1);
    configASSERT(xHandle1);

    // Create task for collecting data from the sensors (pinned to core 1).
    TaskHandle_t xHandle2 = NULL;
    xTaskCreate(testFileCollector, "TEST_CLCTR", 8192, &ucParameterToPass, configMAX_PRIORITIES - 1, &xHandle2);
    // xTaskCreatePinnedToCore(testFileCollector, "TEST_CLCTR", 8192, &ucParameterToPass, configMAX_PRIORITIES - 1, &xHandle2, 0);
    configASSERT(xHandle2);

    // Create tasks to control the LEDs.
    TaskHandle_t xHandle3 = NULL;
    xTaskCreate(BlueLED, "BLUE_LED", 4096, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle3);
    configASSERT(xHandle3);

    TaskHandle_t xHandle4 = NULL;
    xTaskCreate(RedLED, "RED_LED", 4096, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle4);
    configASSERT(xHandle4);

    TaskHandle_t xHandle5 = NULL;
    xTaskCreate(GreenLED, "GREEN_LED", 4096, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle5);
    configASSERT(xHandle5);

    while (1)
    {
        // Give other tasks some time to do stuff.
        vTaskDelay(100 / portTICK_RATE_MS);

        // Do not collect parametric data from the PIC if we are in a test.
        // This is to avoid bogging down the system with useless readings
        // that will have no effect during a test.
        if (testingFirst == false && testing == false)
        {
            // Read ADC data describing the USB_VBUS voltage level.
            uint16_t usb_vbus_fb;
            uint8_t usb_vbus_fb_l, usb_vbus_fb_h;
            i2c_register_read_byte(IPU_I2C_ADDR, 0x10, &usb_vbus_fb_l);
            i2c_register_read_byte(IPU_I2C_ADDR, 0x11, &usb_vbus_fb_h);

            usb_vbus_fb = (usb_vbus_fb_h << 8) | usb_vbus_fb_l;

            // Calculate the USB bus voltage from the raw ADC registers.
            float usb_vbus_voltage = (float)usb_vbus_fb * (5.0 / 4096.0) * 2.0;
            
            // If the USB bus voltage is present and sufficient, update the
            // charger connected state.
            if (usb_vbus_voltage > 4.5) chargerConnected = true;
            else                        chargerConnected = false;

            // Read ADC data describing the V_BATT voltage level.
            uint16_t v_batt_fb;
            uint8_t v_batt_fb_l, v_batt_fb_h;
            i2c_register_read_byte(IPU_I2C_ADDR, 0x12, &v_batt_fb_l);
            i2c_register_read_byte(IPU_I2C_ADDR, 0x13, &v_batt_fb_h);

            v_batt_fb = (v_batt_fb_h << 8) | v_batt_fb_l;

            // Calculate the battery percentage from the raw ADC registers.
            float v_batt_voltage = (float)v_batt_fb * (5.0 / 4096.0) * 2.0;
            batteryPercentage = ((v_batt_voltage - 3.0) / 1.2) * 100.0;

            // Read the button states from the registers.
            i2c_register_read_byte(IPU_I2C_ADDR, 0x0D, &btn1State);
            i2c_register_read_byte(IPU_I2C_ADDR, 0x0E, &btn2State);
            
            // Update the shin strap connected variable.
            i2c_register_read_byte(IPU_I2C_ADDR, 0x16, (uint8_t *)&shinStrapConnected);
        }

        // Introduce hysteresis on the charger.
        // Charge the battery only when the charger is connected and the battery percentage
        // falls below 80%.
        // Add another condition to prevent this from triggering when the device is first
        // powered on. This is becasue the PIC ADC does not convert properly for the first
        // few seconds.
        if (batteryPercentage < 80 && chargerConnected == true && xTaskGetTickCount() > 5000)
        {
            // Enable the charger IC.
            gpio_set_level(GPIO_NUM_17, 0);

            // Set this variable to indicate to the LED task that we are currently charging.
            currentlyCharging = true;
        }

        // Stop charging the battery only if the battery percentage is above 95%.
        if (batteryPercentage > 95)
        {
            // Disable the charger IC.
            gpio_set_level(GPIO_NUM_17, 1);

            // Clear this variable to indicate to the LED task that we are no longer charging.
            currentlyCharging = false;
        }

        // Read any data available from the bluetooth connection.
        if (bt.read_available > 0)
        {
            int len = 512;
            uint8_t data[512];
            bt_read(data, len);

            uint8_t batteryPercentageHundreds = data[0] - 48;
            uint8_t batteryPercentageTens = data[1] - 48;
            uint8_t batteryPercentageOnes = data[2] - 48;
            uint8_t batteryPercentageReceived = (batteryPercentageHundreds * 100) + (batteryPercentageTens * 10) + batteryPercentageOnes;

            uint8_t chargingReceived = data[4] - 48;

            uint8_t shinConnectedReceived = data[6] - 48;

            uint8_t shinMalfunctionReceived = data[8] - 48;

            uint8_t testingReceived = data[10] - 48;

            uint8_t heelLengthTens = data[12] - 48;
            uint8_t heelLengthOnes = data[13] - 48;
            uint8_t heelLengthReceived = (heelLengthTens * 10) + heelLengthOnes;

            bool calibrateStatusReceived = (bool)(data[15] - 48);

            // Print the received data to the terminal.
            printf("statFile recevied from computer:\n");
            printf("  - Battery percentage received: %d\n", batteryPercentageReceived);
            printf("  - Charging received: %d\n", chargingReceived);
            printf("  - Shin connected received: %d\n", shinConnectedReceived);
            printf("  - Shin malfunction received: %d\n", shinMalfunctionReceived);
            printf("  - Testing received: %d\n", testingReceived);
            printf("  - Heel length received: %d\n", heelLengthReceived);
            printf("  - Calibrate status received: %d\n", calibrateStatusReceived);

            // Check if the GUI wants to calibrate the device.
            if (calibrateStatusReceived != calibrateStatus)
            {
                calibrateStatus = calibrateStatusReceived;
                statFileStateChanged = true;
                printf("calibrating load cells.\n");
                calibrateLoadCells();
                printf("calibrate complete.\n");
            }
            
            // Update the global testing variable.
            // testingReceived == true
            if (testingReceived != testing)
            {
                // Reset the iterators for sending and collecting data.
                testFileIter = 0;
                testFileUpTo = 0;
                testingFirst = true;
                statFileStateChanged = true;
                jointAngleCalculatorFirstRun = true;
                printf("start test received from GUI.\n");
            }

            testingFinished = false;
        }

        // Is bluetooth connection established for the first time in this loop?
        if (bt.connected == true && firstConnection == true)
        {
            // Wait for notification subscription.
            if (!bt.notify_enabled) continue;
            printf("Bluetooth connection established (main loop).\n");

            // Send the miscFile as soon as we're connected.
            statFileStateChanged = true;
        }

        if (bt.connected == false)
        {
            // Detect disconnection, so that we can reconnect.
            firstConnection = true;
            // If bluetooth is not connected, figure out if the charger is connected.
            // The PIC will not disable the ESP32 if the charger is connected, no
            // matter the time in the sleep timer. However, the counter will increment
            // to 255, so once the USB cable is disconnected, the device will
            // immediately power off.
            //rc = i2c_register_read_byte(IPU_I2C_ADDR, 31, &chargerConnected);
        }
        else
        {
            // If bluetooth connected, tell the PIC we are not in sleep mode.
            // Write 0 to register 30 to reset sleep state.
            // This register increments on the PIC timer1 counter interrupt and
            // when it reaches 255 then the PIC puts the device into low power mode.
            // (timer period = 1 second, = 255 seconds = 4.25 minutes).
            // rc = i2c_register_write_byte(IPU_I2C_ADDR, 30, 0);
        }

        // Handle the right button being pressed.
        if (btn1State == 1 && btn1Edge == false)
        {
            printf("BTN1 pressed!\n");
            if (testingFirst == false && testing == false)
            {
                // Start the test.
                btn1Edge = true;
                // Reset the iterators for sending and collecting data.
                testFileIter = 0;
                testFileUpTo = 0;
                testingFirst = true;
                jointAngleCalculatorFirstRun = true;
                statFileStateChanged = true;
                printf("start test received from BTN1.\n");
            }
            else
            {
                // Finish the test.
                printf("sending remainder of data from test.\n");
                btn1Edge = true;
                testingFinished = true;
            }
        }
        if (btn1State == 0)
            btn1Edge = false;

        // Handle the left button being pressed.
        if (btn2State == 1 && btn2Edge == false)
        {
            printf("BTN2 pressed!\n");
            if (testingFirst == false && testing == false)
            {
                // Start the test.
                btn2Edge = true;
                // Reset the iterators for sending and collecting data.
                testFileIter = 0;
                testFileUpTo = 0;
                testingFinished = false;
                testingFirst = true;
                jointAngleCalculatorFirstRun = true;
                statFileStateChanged = true;
                printf("start test received from BTN2.\n");
            }
            else
            {
                // Finish the test.
                printf("sending remainder of data from test.\n");
                btn2Edge = true;
                testingFinished = true;
            }
        }
        if (btn2State == 0)
            btn2Edge = false;

        if (statFileStateChanged == true)
        {
            // Send the statFile when one of the variables changed.
            send_statFile();
            // Don't send the statFile twice.
            statFileStateChanged = false;

            // Update the firstConnection status, because statFile is sent
            // when the device connects for the first time.
            firstConnection = false;

            // Reset the calibrate status if it was running.
            calibrateStatus = false;

            if (testingFirst == true) testing = true;
            testingFirst = false;
        }
    }
}
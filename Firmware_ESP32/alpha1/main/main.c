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

#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          (46) // Define the output GPIO
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define LEDC_DUTY               (4095) // Set duty to 50%. ((2 ** 13) - 1) * 50% = 4095
#define LEDC_FREQUENCY          (5000) // Frequency in Hertz. Set frequency at 5 kHz


// Initialize the bluetooth module.
struct bt_config_t bt = 
{
    .device_name = "STRASAnkle Gamma"
};

struct kalman_filter fkp;
struct kalman_filter fkr;
struct kalman_filter skp;
struct kalman_filter skr;

int8_t calibrateStatus = 2;

// Send the miscFile to the GUI.
int send_miscFile(struct bt_config_t *bt, uint8_t start_test)
{
    if (bt->connected == false) return -1;

    uint8_t case1[11];

    while (bt->write_available == false)
    {
        if (bt->connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    strcpy((char *)case1, "statFile,w");
    if (bt_notify(case1, 10) != 0) return -1;

    while (bt->write_available == false)
    {
        if (bt->connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    strcpy((char *)case1, "statFile");
    if (bt_write(case1, 8) != 0) return -1;

    case1[0] = 33;
    case1[1] = 0;
    case1[2] = 0;
    case1[3] = 1;
    case1[4] = start_test;
    case1[5] = 0;
    case1[6] = calibrateStatus;
    while (bt->write_available == false)
    {
        if (bt->connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    if (bt_write(case1, 7) != 0) return -1;

    strcpy((char *)case1, "EOF");
    while (bt->write_available == false)
    {
        if (bt->connected == false) return -1;
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    if (bt_write(case1, 3) != 0) return -1;

    return 0;
}

// ADC instance.
spi_device_handle_t adc;

// Detect button edge press.
bool btn1Edge = false;
bool btn2Edge = false;

// Detect if the bluetooth was connected for the first time in the main loop.
bool firstConnection = true;

// Do we want to send the testFile?
bool sendTestFile = false;
// First pass of sending the testFile? (Send notification & initial packet).
bool sendTestFileFirst = true;
uint8_t *testFileData = NULL;
int32_t testFileIter = 0;
int32_t testFileUpTo = 0;

// Is the charger connected? We need this variable because it determines the LED states.
uint8_t chargerConnected = 0;

#define PI          (3.14159265359)
#define PI2         (6.28318530718)
#define PI180       (0.01745329251)
#define PI180I      (57.2957795131)

bool jointAngleCalculatorFirstRun = true;

// foot 
double f_gyroXangle, f_gyroYangle; // Angle calculate using the gyro only
double f_compAngleX, f_compAngleY; // Calculated angle using a complementary filter
double f_kalAngleX, f_kalAngleY;   // Calculated angle using a Kalman filter
// shin
double s_gyroXangle, s_gyroYangle; // Angle calculate using the gyro only
double s_compAngleX, s_compAngleY; // Calculated angle using a complementary filter
double s_kalAngleX, s_kalAngleY;   // Calculated angle using a Kalman filter

void jointAngleCalculator(float f_acc_x, float f_acc_y, float f_acc_z,
                         float f_gyr_x, float f_gyr_y, float f_gyr_z,
                         float s_acc_x, float s_acc_y, float s_acc_z,
                         float s_gyr_x, float s_gyr_y, float s_gyr_z,
                         double dt,
                         float *r, float *p)
{
    double f_roll = atan(f_acc_y / sqrt(f_acc_x * f_acc_x + f_acc_z * f_acc_z)) * PI180I;   //fooot
    double f_pitch = atan2(-f_acc_x, f_acc_z) * PI180I;  
    double s_roll = atan(s_acc_y / sqrt(s_acc_x * s_acc_x + s_acc_z * s_acc_z)) * PI180I;   //shin
    double s_pitch = atan2(-s_acc_x, s_acc_z) * PI180I;

    double f_gyroXrate = (double)f_gyr_x; // deg/s    //fooot
    double f_gyroYrate = (double)f_gyr_y; // deg/s
    double s_gyroXrate = (double)s_gyr_x; // deg/s    //shin
    double s_gyroYrate = (double)s_gyr_y; // deg/s

    if (jointAngleCalculatorFirstRun == true) 
    {
        // Set starting angle
        kalman_setAngle(&fkr, f_roll); //foot
        kalman_setAngle(&fkp, f_pitch);
        kalman_setAngle(&skr, s_roll); // shin
        kalman_setAngle(&skp, s_pitch);

        // Foot.
        f_gyroXangle = f_roll;
        f_gyroYangle = f_pitch;
        f_compAngleX = f_roll;
        f_compAngleY = f_pitch;
        // Shin.
        s_gyroXangle = s_roll;
        s_gyroYangle = s_pitch;
        s_compAngleX = s_roll;
        s_compAngleY = s_pitch;

        jointAngleCalculatorFirstRun = false;
    }

    /////foot
    // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
    if ((f_pitch < -90.0 && f_kalAngleY > 90.0) || (f_pitch > 90.0 && f_kalAngleY < -90.0))
    {
        kalman_setAngle(&fkp, f_pitch);
        f_compAngleY = f_pitch;
        f_kalAngleY = f_pitch;
        f_gyroYangle = f_pitch;
    }
    else
        f_kalAngleY = kalman_getAngle(&skp, f_pitch, f_gyroYrate, dt); // Calculate the angle using a Kalman filter

    if (abs(f_kalAngleY) > 90.0)
        f_gyroXrate = -f_gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
    f_kalAngleX = kalman_getAngle(&fkr, f_roll, f_gyroXrate, dt); // Calculate the angle using a Kalman filter

    /////shin
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


    ///foot
    f_gyroXangle += f_gyroXrate * dt; // Calculate gyro angle without any filter
    f_gyroYangle += f_gyroYrate * dt;
    //f_gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //f_gyroYangle += kalmanY.getRate() * dt;
    ///shin
    s_gyroXangle += s_gyroXrate * dt; // Calculate gyro angle without any filter
    s_gyroYangle += s_gyroYrate * dt;
    //f_gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
    //f_gyroYangle += kalmanY.getRate() * dt;

    ////foot
    f_compAngleX = 0.93 * (f_compAngleX + f_gyroXrate * dt) + 0.07 * f_roll; // Calculate the angle using a Complimentary filter
    f_compAngleY = 0.93 * (f_compAngleY + f_gyroYrate * dt) + 0.07 * f_pitch;
    ///shin
    s_compAngleX = 0.93 * (s_compAngleX + s_gyroXrate * dt) + 0.07 * s_roll; // Calculate the angle using a Complimentary filter
    s_compAngleY = 0.93 * (s_compAngleY + s_gyroYrate * dt) + 0.07 * s_pitch;


    ///foot
    // Reset the gyro angle when it has drifted too much
    if (f_gyroXangle < -180.0 || f_gyroXangle > 180.0)
        f_gyroXangle = f_kalAngleX;
    if (f_gyroYangle < -180.0 || f_gyroYangle > 180.0)
        f_gyroYangle = f_kalAngleY;
    ///shin
    if (s_gyroXangle < -180.0 || s_gyroXangle > 180.0)
        s_gyroXangle = s_kalAngleX;
    if (s_gyroYangle < -180.0 || s_gyroYangle > 180.0)
        s_gyroYangle = s_kalAngleY;

    // printf("%.2f\t%.2f\t", f_kalAngleX, f_kalAngleY);//foot
    // printf("%.2f\t%.2f\n", s_kalAngleX, s_kalAngleY);//shin
}

void testFileBTSender(void * pvParameters)
{
    printf("testFileBTSenderTask started.\n");
    while (1)
    {
        // Give time to other tasks.
        vTaskDelay(100 / portTICK_RATE_MS);
        // If not actively sending test file.
        if (sendTestFile == false) continue;

        // Initialize variables for sending text.
        uint8_t case1[11];

        // First pass, allocate memory and send required stuff.
        if (sendTestFileFirst == true)
        {
            // Check if the device is connected. If not, don't bother sending the test.
            if (bt.connected == false) continue;

            // Notify the GUI.
            while (bt.write_available == false) vTaskDelay(10 / portTICK_RATE_MS);
            strcpy((char *)case1, "testFile,w");
            bt_notify(case1, 10);

            // Initial packet.
            while (bt.write_available == false) vTaskDelay(10 / portTICK_RATE_MS);
            strcpy((char *)case1, "testFile");
            bt_write(case1, 8);

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
                sendTestFile = false;
                continue;
            }
            testFileIter = 0;
            testFileUpTo = 0;
            sendTestFileFirst = false;
        }

        // Allocated memory, now we can collect and send data.
        while (bt.write_available == false ||           // The GUI hasn't read yet.
               testFileIter >= testFileUpTo - 512)      // Data collector task hasn't collected data for the next frame yet.
               vTaskDelay(10 / portTICK_RATE_MS);

        printf("testFileIter = %d, testFileUpTo = %d, data[%d] = %02X\n", testFileIter, testFileUpTo, testFileIter, testFileData[testFileIter]);

        // Send the data the other task just read.
        bt_write(&testFileData[testFileIter], 512);

        // Increment this task's file pointer indicating we are past where the data collection task has reached.
        testFileIter += 512;

        // Debug test file iterator as we move through the file.
        printf("testFileIter = %d.\n", testFileIter);

        // Stop after a certian point.
        if (testFileIter > 5000)
        {
            strcpy((char *)case1, "EOF");
            while (bt.write_available == false) vTaskDelay(10 / portTICK_RATE_MS);
            bt_write((uint8_t *)case1, 3);
            sendTestFile = false;
            sendTestFileFirst = true;
            printf("finished sending dataFile.\n");
            // Don't forget to free memory.
            if (testFileData)
                free(testFileData);
        }
    }
}

void testFileCollector(void * pvParameters)
{
    printf("testFileCollectorTask started.\n");
    // Initial RTOS time in ms.
    int dt = xTaskGetTickCount() * 10;
    int prevTime = xTaskGetTickCount() * 10;
    int16_t t = 0;
    while (1)
    {
        dt = (xTaskGetTickCount() * 10) - prevTime;
        t += dt;
        prevTime = xTaskGetTickCount() * 10;

        // Give time to other tasks.
        //vTaskDelay(100 / portTICK_RATE_MS);
        vTaskDelay(10 / portTICK_RATE_MS);
        // If not actively sending test file.
        // sendTestFileFirst will go false once the first pass complete (malloc done).
        // Also check for malloc successful.
        if (sendTestFile == false || sendTestFileFirst == true || testFileData == NULL) continue;

        // Initialize data to store the shin strap data.
        uint8_t data[12];

        // Read the shin strap data.
        for (int i = 0; i < 12; i++)
        {
            // int rc = i2c_register_read_byte(IPU_I2C_ADDR, i, &data[i]);
            // if (rc != 0)
            // {
            //     printf("error:%d\n", rc);
            //     // Re-read the data from the shinstrap.
            //     i--;
            // }
        }

        // Read the ISM330 data.
        // float sgyrx = ism330_convert_gyr_x_dps((data[1] << 8) | data[0]);
        // float sgyry = ism330_convert_gyr_y_dps((data[3] << 8) | data[2]);
        // float sgyrz = ism330_convert_gyr_z_dps((data[5] << 8) | data[4]);
        // float saccx = ism330_convert_acc_x_g((data[7] << 8) | data[6]);
        // float saccy = ism330_convert_acc_y_g((data[9] << 8) | data[8]);
        // float saccz = ism330_convert_acc_z_g((data[11] << 8) | data[10]);

        // float faccx = ism330_get_acc_x_g();
        // float faccy = ism330_get_acc_y_g();
        // float faccz = ism330_get_acc_z_g();

        // float fgyrx = ism330_get_gyr_x_dps();
        // float fgyry = ism330_get_gyr_y_dps();
        // float fgyrz = ism330_get_gyr_z_dps();

        // Stop automatically.
        if (testFileUpTo > 6000) continue;

        // float roll = 0.0;
        // float pitch = 0.0;

        // jointAngleCalculator(faccx, faccy, faccz, fgyrx, fgyry, fgyrz, saccx, saccy, saccz, sgyrx, sgyry, sgyrz, (double)dt, &roll, &pitch);

        // int16_t roll2 = (int16_t)(roll * 364.0);
        // int16_t pitch2 = (int16_t)(pitch * 364.0);

        // Update the testFileData and increment the upTo indexer for the other task.

        // Send current time.
        testFileData[testFileUpTo++] = 0;//(t >> 8) & 0xff;
        testFileData[testFileUpTo++] = 0;//t & 0xff;

        // Send pitch data with scale factor.
        testFileData[testFileUpTo++] = 0;//(pitch2 >> 8) & 0xff;
        testFileData[testFileUpTo++] = 0;//pitch2 & 0xff;

        // Send roll data with scale factor.
        testFileData[testFileUpTo++] = 0;//(roll2 >> 8) & 0xff;
        testFileData[testFileUpTo++] = 0;//roll2 & 0xff;

        // Send load cell data with scale factor.
        testFileData[testFileUpTo++] = 0;
        testFileData[testFileUpTo++] = 1;
    }
}

void calibrateLoadCells(void)
{
    double ch0v, ch1v;
    double cal0 = 0.0, cal1 = 0.0;
    double sar = 1.0;

    // Define where we want the reading to settle (volts).
    const double mid = 2.5;

    // Calibrate the channels using successive approximation.
    while (sar > 0.00001)
    {
        // Set the new calibration values.
        dac_write(cal0, cal1);

        // printf("calibration values: %.4f, %.4f\n", cal0, cal1);

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

        // printf("ADC reading: %.4f, %.4f\n", ch0v, ch1v);

        if (ch0v > mid) cal1 -= sar;
        else            cal1 += sar;
        if (ch1v > mid) cal0 -= sar;
        else            cal0 += sar;
        sar /= 2.0;
    }

    printf("final calibration values: %.4f, %.4f\n", cal0, cal1);

    // We don't need to continuously write to the DAC, so this function can just return.
}


void BlueLED(void)
{
    while (1)
    {
        if (bt.initialized == true && chargerConnected == 0)
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

void GreenLED(void)
{
    // Full on = 8191, off = 0.

    int16_t pulser = 0;
    int8_t dir = 1;

    while (1)
    {
        if (sendTestFile == true)
        {
            // Test has started.
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 8191);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(250 / portTICK_RATE_MS);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(250 / portTICK_RATE_MS);
        }
        else if (calibrateStatus == 1)
        {
            gpio_set_level(GPIO_NUM_45, 1);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(250 / portTICK_RATE_MS);
            gpio_set_level(GPIO_NUM_45, 0);
            ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 8191);
            ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
            vTaskDelay(250 / portTICK_RATE_MS);
        }
        else
        {
            // nCHG_OK = 0 (charging active).
            if (gpio_get_level(GPIO_NUM_39) == 0)
            {
                if (chargerConnected == 1)
                {
                    // Actively charging the battery (pulsing green LED).
                    // Direction change for pulse.
                    if (pulser >= 8191) dir = -1;
                    if (pulser <= 0) dir = 1;

                    pulser += 100 * dir;

                    // Protect edge cases.
                    if (pulser >= 8191) pulser = 8191;
                    if (pulser <= 0) pulser = 0;

                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, pulser);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(20 / portTICK_RATE_MS);
                }
                else
                {
                    // Battery not charging, and test not in progress.
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(1000 / portTICK_RATE_MS);
                }
            }
            else
            {
                if (chargerConnected == 1)
                {
                    // Charging completed.
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 8191);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(1000 / portTICK_RATE_MS);
                }
                else
                {
                    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
                    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
                    vTaskDelay(1000 / portTICK_RATE_MS);
                }
            }
        }
    }
}



// /* MAIN CODE

void app_main(void)
{
    // Set LED1 as output.
    gpio_config_t LED1 = 
    {
        .pin_bit_mask = GPIO_NUM_38,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&LED1);
    gpio_set_direction(GPIO_NUM_38, GPIO_MODE_OUTPUT);

    // Set LED2 as output.
    gpio_config_t LED2 = 
    {
        .pin_bit_mask = GPIO_NUM_45,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&LED2);
    gpio_set_direction(GPIO_NUM_45, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_45, 1);

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

        // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_MODE,
        .timer_num        = LEDC_TIMER,
        .duty_resolution  = LEDC_DUTY_RES,
        .freq_hz          = LEDC_FREQUENCY,  // Set output frequency at 5 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel = {
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = LEDC_OUTPUT_IO,
        .duty           = 0, // Set duty to 0%
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel));


    // Initialize the sensors.
    int rc = i2c_master_init(36, 37, 100000);
    printf("I2C init return code: %d\n", rc);

    ism330_init();
    printf("ISM330 init.\n");

    bt_init(&bt);
    printf("BT init return.\n");

    // Initialize ADC.
    adc_init(&adc, 7);
    printf("ADC initialized.\n");

    // Initialize DAC. Use a high sampling rate for now minimize avoid noise on the output (how to avoid this?).
    dac_init(96000);
    printf("DAC initialized.\n");

    kalman_init(&fkp);
    kalman_init(&fkr);
    kalman_init(&skp);
    kalman_init(&skr);


    // Create task for sending data.
    TaskHandle_t xHandle1 = NULL;
    static uint8_t ucParameterToPass = 0;
    xTaskCreatePinnedToCore(testFileBTSender, "TESTFILE_SNDR", 8192, &ucParameterToPass, configMAX_PRIORITIES-1, &xHandle1, 0);
    configASSERT(xHandle1);

    // Create task for collecting data.
    TaskHandle_t xHandle2 = NULL;
    xTaskCreatePinnedToCore(testFileCollector, "TEST_CLCTR", 8192, &ucParameterToPass, configMAX_PRIORITIES-1, &xHandle2, 1);
    configASSERT(xHandle2);

    TaskHandle_t xHandle3 = NULL;
    xTaskCreate(BlueLED, "BLUE_LED", 4096, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle3);
    configASSERT(xHandle3);

    TaskHandle_t xHandle4 = NULL;
    xTaskCreate(GreenLED, "GREEN_LED", 4096, &ucParameterToPass, tskIDLE_PRIORITY + 1, &xHandle4);
    configASSERT(xHandle4);


    calibrateLoadCells();


    while (1)
    {
        // Give other tasks some time to do stuff.
        // vTaskDelay(100 / portTICK_RATE_MS);


        double ch0v = 0.0;
        double ch1v = 0.0;

        for (uint8_t i = 0; i < 10; i++)
        {
            double ch0tmp;
            double ch1tmp;
            adc_read_voltage(&adc, &ch0tmp, &ch1tmp);
            ch0v += ch0tmp;
            ch1v += ch1tmp;
            vTaskDelay(1);
        }

        ch0v *= 10000;
        ch1v *= 10000;

        printf("%.1f\t%.1f\n", ch0v, ch1v);
        

        int rc = 0;
        uint16_t usb_vbus_fb;
        uint8_t usb_vbus_fb_l, usb_vbus_fb_h;
        rc = i2c_register_read_byte(IPU_I2C_ADDR, 0x10, &usb_vbus_fb_l);
        // printf("rc = %d\n", rc);
        rc = i2c_register_read_byte(IPU_I2C_ADDR, 0x11, &usb_vbus_fb_h);
        // printf("rc = %d\n", rc);

        usb_vbus_fb = (usb_vbus_fb_h << 8) | usb_vbus_fb_l;

        float usb_vbus_voltage = (float)usb_vbus_fb * (5.0 / 4096.0) * 2.0;

        // printf("raw adc value: %04X\n", usb_vbus_fb);
        // printf("usb vbus voltage: %.2f\n", usb_vbus_voltage);

        uint16_t v_batt_fb;
        uint8_t v_batt_fb_l, v_batt_fb_h;
        rc = i2c_register_read_byte(IPU_I2C_ADDR, 0x12, &v_batt_fb_l);
        // printf("rc = %d\n", rc);
        rc = i2c_register_read_byte(IPU_I2C_ADDR, 0x13, &v_batt_fb_h);
        // printf("rc = %d\n", rc);

        v_batt_fb = (v_batt_fb_h << 8) | v_batt_fb_l;

        float v_batt_voltage = (float)v_batt_fb * (5.0 / 4096.0) * 2.0;

        // printf("raw adc value: %04X\n", v_batt_fb);
        // printf("v_batt voltage: %.2f\n", v_batt_voltage);

        // Detect charger plugged in, and if so, enable nCHG_EN.
        if (chargerConnected == 1)
        {
            gpio_set_level(GPIO_NUM_17, 0);
        }
        else
        {
            gpio_set_level(GPIO_NUM_17, 1);
        }

        // TESTING THE LOAD CELLS
        // uint32_t ch0Val, ch1Val;
        //adc_read_raw(&adc, &ch0Val, &ch1Val);

        // printf("%06X\t%06X\n", ch0Val, ch1Val);
        // printf("%d\n", ch0Val);

        // Is data available from the bluetooth connection?
        // TODO: Implement this later.
        if (bt.read_available > 0)
        {
            int len = 512;
            uint8_t data[512];
            bt_read(data, len);

            printf("data = %.*s\n", 50, data);

            if (calibrateStatus == 2)
            {
                
                // char *ptr = (char *)&data[0];
                // for (int i = 0; i < 6; i++)
                // {
                //     ptr = strstr(ptr, ",");
                //     printf("index = %d\n", i);
                //     if (ptr == NULL)
                //     {
                //         printf("null pointer");
                //         break;
                //     }
                // }

                // printf("%s\n", data);

                calibrateStatus = 1;// (uint8_t)*(ptr + 1) - 48;

                printf("calibrate status: %d\n", calibrateStatus);

                vTaskDelay(5000 / portTICK_RATE_MS);

                printf("calibrate complete.\n");

                calibrateStatus = 0;

                send_miscFile(&bt, 0);

                gpio_set_level(GPIO_NUM_45, 1);
            }
            else
            {
                sendTestFile = true;
                jointAngleCalculatorFirstRun = true;
            }
        }

        // Is bluetooth connection established for the first time in this loop?
        if (bt.connected == true && firstConnection == true)
        {
            // Wait for notification subscription.
            if (!bt.notify_enabled) continue;
            printf("Bluetooth connection established (main loop).\n");

            // Send the miscFile as soon as we're connected.
            // If sending the miscFile fails, don't update the firstConnection state.
            if (send_miscFile(&bt, 0) == 0)
            {
                firstConnection = false;
            }
        }

        if (bt.connected == false)
        {
            // Detect disconnection, so that we can reconnect.
            firstConnection = true;
            // If bluetooth not connected, figure out if the charger is connected.
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

        // Read the button states.
        uint8_t btn1State = 0;
        uint8_t btn2State = 0;
        // rc = i2c_register_read_byte(IPU_I2C_ADDR, 20, &btn1State);
        if (rc != ESP_OK)
            printf("rc = %d\n", rc);
        // rc = i2c_register_read_byte(IPU_I2C_ADDR, 21, &btn2State);
        if (rc != ESP_OK)
            printf("rc = %d\n", rc);

        // Handle start test button being pressed.
        if (btn1State == 1 && btn1Edge == false)
        {
            printf("BTN1 pressed!\n");
            printf("sending miscFile with start test.\n");
            send_miscFile(&bt, 1);
            btn1Edge = true;
            sendTestFile = true;
            jointAngleCalculatorFirstRun = true;
        }
        if (btn1State == 0)
            btn1Edge = false;

        // Handle stop test button being pressed.
        if (btn2State == 1 && btn2Edge == false)
        {
            printf("BTN2 pressed!\n");
            printf("sending miscFile with stop test.\n");
            send_miscFile(&bt, 0);
            btn2Edge = true;
            sendTestFile = false;
        }
        if (btn2State == 0)
            btn2Edge = false;
    }
}

// */

/* JOINT ANGLE ALGORITHM CHECK


void app_main(void)
{
    // Initialize the sensors.
    int rc = i2c_master_init(36, 37, 100000);
    printf("I2C init return code: %d\n", rc);

    ism330_init();
    printf("ISM330 init.\n");

    while (1)
    {
        // Initialize data to store the shin strap data.
        uint8_t data[12];

        // Read the shin strap data.
        for (int i = 0; i < 12; i++)
        {
            int rc = i2c_register_read_byte(IPU_I2C_ADDR, i, &data[i]);
            if (rc != 0)
                printf("error:%d\n", rc);
        }

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

        float t = (float)(xTaskGetTickCount());

        float roll, pitch;

        // printf("%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\n", faccx, faccy, faccz, fgyrx, fgyry, fgyrz, saccx, saccy, saccz, sgyrx, sgyry, sgyrz);

        jointAngleCalculator(faccx, faccy, faccz, fgyrx, fgyry, fgyrz, saccx, saccy, saccz, sgyrx, sgyry, sgyrz, t, &roll, &pitch);

        //printf("%.2f\t%.2f\n", roll, pitch);
    }

}

*/


/* IMU CHECK

void app_main(void)
{
    ESP_ERROR_CHECK(i2c_master_init(36, 37, 400000));
    printf("I2C initialized successfully\n");

    ism330_init();

    while (1)
    {
        float accx = ism330_get_acc_x_g();
        float accy = ism330_get_acc_y_g();
        float accz = ism330_get_acc_z_g();
        float gyrx = ism330_get_gyr_x_dps();
        float gyry = ism330_get_gyr_y_dps();
        float gyrz = ism330_get_gyr_z_dps();

        float pitch = accx;
        float roll = accy;

        pitch *= 364;
        roll *= 364;

        int16_t pitch16 = (int16_t)pitch;
        int16_t roll16 = (int16_t)roll;

        // float temp = ism330_get_temp_celcius();
        
        printf("%d, %d, %d, 0,\n", xTaskGetTickCount(), pitch16, roll16);




        vTaskDelay(10 / portTICK_RATE_MS);
    }

    ESP_ERROR_CHECK(i2c_driver_delete(0));
    printf("I2C de-initialized successfully\n");
}

*/

/* BLUETOOTH CHECK

void app_main(void)
{
    // Initialize the bluetooth module.
    struct bt_config_t bt = 
    {
        .device_name = "STRASAnkle Gamma"
    };
    bt_init(&bt);

    setvbuf(stdin, NULL, _IONBF, 0);
    setvbuf(stdout, NULL, _IONBF, 0);
    ESP_ERROR_CHECK(uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0));
    esp_vfs_dev_uart_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    esp_vfs_dev_uart_port_set_rx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CR);
    esp_vfs_dev_uart_port_set_tx_line_endings(CONFIG_ESP_CONSOLE_UART_NUM, ESP_LINE_ENDINGS_CRLF);
    char buf[10];

    while (1)
    {
        scanf("%1s", buf);

        printf("received: %s\n", buf);

        if (bt.connected == true && bt.write_available == true && buf[0] == '2')
        {
            uint8_t bigFile[512];

            while (bt.write_available == false);
            strcpy((char *)bigFile, "testFile,w");
            bt_notify(bigFile, 10);

            while (bt.write_available == false);
            strcpy((char *)bigFile, "testFile");
            bt_write(bigFile, 8);

            size_t sz = sizeof(sample_dataset) / sizeof(uint16_t);

            printf("total size = %d\n", sz);

            size_t curPos = 0;

            while (curPos < sz - 256)
            {
                while (bt.write_available == false);
                printf("bt_write returns: %d\n", bt_write((uint8_t *)&sample_dataset[curPos], 512));
                curPos += 256;
            }

            size_t rem = sz - curPos;
            while (bt.write_available == false);
            printf("bt_write returns: %d\n", bt_write((uint8_t *)&sample_dataset[curPos], rem * 2));

            // EOF character.
            strcpy((char *)bigFile, "EOF");
            while (bt.write_available == false);
            bt_write((uint8_t *)bigFile, 3);
        }

        if (bt.connected == true && bt.write_available == true && buf[0] == '1')
        {
            uint8_t case1[11];

            while (bt.write_available == false);
            strcpy((char *)case1, "statFile,w");
            bt_notify(case1, 10);

            while (bt.write_available == false);
            strcpy((char *)case1, "statFile");
            bt_write(case1, 8);

            case1[0] = 63;
            case1[1] = 0;
            case1[2] = 0;
            case1[3] = 1;
            case1[4] = 1;
            case1[5] = 0;
            while (bt.write_available == false);
            bt_write(case1, 6);

            strcpy((char *)case1, "EOF");
            while (bt.write_available == false);
            bt_write((uint8_t *)case1, 3);
        }

        if (bt.connected == true && bt.write_available == true && buf[0] == '0')
        {
            uint8_t case2[11];

            while (bt.write_available == false);
            strcpy((char *)case2, "statFile,w");
            bt_notify(case2, 10);

            while (bt.write_available == false);
            strcpy((char *)case2, "statFile");
            bt_write(case2, 9);

            case2[0] = 63;
            case2[1] = 0;
            case2[2] = 0;
            case2[3] = 1;
            case2[4] = 0;
            case2[5] = 0;
            while (bt.write_available == false);
            bt_write(case2, 6);

            strcpy((char *)case2, "EOF");
            while (bt.write_available == false);
            bt_write((uint8_t *)case2, 3);
        }
        // if (bt.connected == true && bt.read_available == true)
        // {
        //     bt_read(data, len);
        //     printf("read characteristic: %02X\n", data[0]);
        // }
        // if (bt.notify_enabled == true)
        // {
        //     char data[] = "notify test.";
        //     bt_notify((uint8_t *)data, 12);
        // }
        // vTaskDelay(100 / portTICK_RATE_MS);
        // vTaskDelay(1 / portTICK_RATE_MS);
    }

}

*/

/* LOAD CELL CHECK

void app_main(void)
{
    // Initialize ADC.
    spi_device_handle_t adc;
    adc_init(&adc, 7);
    printf("ADC initialized.\n");

    // Initialize DAC. Use a high sampling rate for now to avoid noise on the output (how to avoid this?).
    dac_init(192000);
    printf("DAC initialized.\n");

    // Enable 5VA.
    gpio_config_t nVDD_5VA_EN = 
    {
        .pin_bit_mask = GPIO_NUM_35,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&nVDD_5VA_EN);
    gpio_set_direction(GPIO_NUM_35, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_35, 1);

    // ESP32 LED.
    gpio_config_t ESP32_LED = 
    {
        .pin_bit_mask = GPIO_NUM_9,
        .mode = GPIO_MODE_OUTPUT,
    };
    gpio_config(&ESP32_LED);
    gpio_set_direction(GPIO_NUM_9, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_9, 0);

    // // Calibrate
    // float i = 0.0;

    // while (1)
    // {
    //     uint32_t ch0Val, ch1Val;
    //     adc_read_raw(&adc, 7, &ch0Val, &ch1Val);

    //     gpio_set_level(GPIO_NUM_9, 1);
    //     printf("%d\t%d\t%.2f\n", ch0Val, ch1Val, i);
    //     vTaskDelay(100 / portTICK_RATE_MS);

    //     dac_write(i, i);

    //     i = i + 0.01;
    //     if (i > 1.0) i = -1.0;
    // }

    float i = 0.13; // BACK CHANNEL
    // float i = 0.03; // FRONT CHANNEL

    dac_write(0.0, 0.0);

    gpio_set_level(GPIO_NUM_9, 1);

    uint32_t ch0Val, ch1Val;

    while (1)
    {
        adc_read_raw(&adc, 7, &ch0Val, &ch1Val);
        printf("%08X\t\t%08X\n", ch0Val, ch1Val);
        vTaskDelay(10 / portTICK_RATE_MS);
    }
}

*/
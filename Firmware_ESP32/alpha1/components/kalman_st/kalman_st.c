#include "kalman_st.h"

void kalman_init(struct kalman_filter *k)
{
    /* We will set the variables like so, these can also be tuned by the user */
    k->kalman_Q_angle = 0.001f;
    k->kalman_Q_bias = 0.003f;
    k->kalman_R_measure = 0.03f;

    k->kalman_angle = 0.0f; // Reset the angle
    k->kalman_bias = 0.0f; // Reset bias

    k->kalman_P[0][0] = 0.0f; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en.wikipedia.org/wiki/Kalman_filter#Example_application.2C_technical
    k->kalman_P[0][1] = 0.0f;
    k->kalman_P[1][0] = 0.0f;
    k->kalman_P[1][1] = 0.0f;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalman_getAngle(struct kalman_filter *k, float newAngle, float newRate, float dt)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    k->kalman_rate = newRate - k->kalman_bias;
    k->kalman_angle += dt * k->kalman_rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    k->kalman_P[0][0] += dt * (dt*k->kalman_P[1][1] - k->kalman_P[0][1] - k->kalman_P[1][0] + k->kalman_Q_angle);
    k->kalman_P[0][1] -= dt * k->kalman_P[1][1];
    k->kalman_P[1][0] -= dt * k->kalman_P[1][1];
    k->kalman_P[1][1] += k->kalman_Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    float S = k->kalman_P[0][0] + k->kalman_R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = k->kalman_P[0][0] / S;
    K[1] = k->kalman_P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    float y = newAngle - k->kalman_angle; // Angle difference
    /* Step 6 */
    k->kalman_angle += K[0] * y;
    k->kalman_bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = k->kalman_P[0][0];
    float P01_temp = k->kalman_P[0][1];

    k->kalman_P[0][0] -= K[0] * P00_temp;
    k->kalman_P[0][1] -= K[0] * P01_temp;
    k->kalman_P[1][0] -= K[1] * P00_temp;
    k->kalman_P[1][1] -= K[1] * P01_temp;

    return k->kalman_angle;
}

void kalman_setAngle(struct kalman_filter *k,float angle)
{
    k->kalman_angle = angle;
} // Used to set angle, this should be set as the starting angle

float kalman_getRate(struct kalman_filter *k)
{
    return k->kalman_rate;
} // Return the unbiased rate

/* These are used to tune the Kalman filter */
void kalman_setQangle(struct kalman_filter *k,float Q_angle)
{
     k->kalman_Q_angle = Q_angle;
}

void kalman_setQbias(struct kalman_filter *k,float Q_bias)
{
    k->kalman_Q_bias = Q_bias;
}

void kalman_setRmeasure(struct kalman_filter *k,float R_measure)
{
    k->kalman_R_measure = R_measure;
}

float kalman_getQangle(struct kalman_filter *k)
{
    return k->kalman_Q_angle;
}
float kalman_getQbias(struct kalman_filter *k)
{
    return k->kalman_Q_bias;
}

float kalman_getRmeasure(struct kalman_filter *k)
{
    return k->kalman_R_measure;
}
#ifndef kalman_st_h
#define kalman_st_h

struct kalman_filter
{
  /* Kalman filter variables */
  float kalman_Q_angle; // Process noise variance for the accelerometer
  float kalman_Q_bias; // Process noise variance for the gyro bias
  float kalman_R_measure; // Measurement noise variance - this is actually the variance of the measurement noise

  float kalman_angle; // The angle calculated by the Kalman filter - part of the 2x1 state vector
  float kalman_bias; // The gyro bias calculated by the Kalman filter - part of the 2x1 state vector
  float kalman_rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

  float kalman_P[2][2]; // Error covariance matrix - This is a 2x2 matrix
};

void kalman_init(struct kalman_filter *k);

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float kalman_getAngle(struct kalman_filter *k, float newAngle, float newRate, float dt);

void kalman_setAngle(struct kalman_filter *k,float angle);// Used to set angle, this should be set as the starting angle
float kalman_getRate(struct kalman_filter *k);// Return the unbiased rate

/* These are used to tune the Kalman filter */
void kalman_setQangle(struct kalman_filter *k,float Q_angle);
/**
 * setQbias(float Q_bias)
 * Default value (0.003f) is in Kalman.cpp. 
 * Raise this to follow input more closely,
 * lower this to smooth result of kalman filter.
 */
void kalman_setQbias(struct kalman_filter *k,float Q_bias);
void kalman_setRmeasure(struct kalman_filter *k,float R_measure);

float kalman_getQangle(struct kalman_filter *k);
float kalman_getQbias(struct kalman_filter *k);
float kalman_getRmeasure(struct kalman_filter *k);

#endif
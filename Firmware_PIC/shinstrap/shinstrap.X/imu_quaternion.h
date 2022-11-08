#ifndef IMU_QUATERNION_H
#define IMU_QUATERNION_H

#include <xc.h> // include processor files - each processor file is guarded.  

void mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float deltat);

#endif	/* XC_HEADER_TEMPLATE_H */


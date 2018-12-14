#ifndef _QUATERNIONFILTERS_H_
#define _QUATERNIONFILTERS_H_

void madgwick_quaternion_update(float ax, float ay, float az, float gx, float gy,
                              float gz, float mx, float my, float mz,
                              float deltat);
void mahony_quaternion_update(float ax, float ay, float az, float gx, float gy,
                            float gz, float mx, float my, float mz,
                            float deltat);
const float * get_q();

#endif // _QUATERNIONFILTERS_H_

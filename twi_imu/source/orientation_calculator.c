#include <math.h>
#include "orientation_calculator.h"

//Math Help
#define M_PI 3.14159265358979323846
#define RAD_TO_DEG (180.0f / M_PI)

static orientation_result_t m_mpu_result; //Yaw, Pitch and Roll

const orientation_result_t * calculate_orientation(const float * q)
{
    m_mpu_result.yaw   = RAD_TO_DEG * atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                                            q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    m_mpu_result.pitch = RAD_TO_DEG * -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    m_mpu_result.roll  = RAD_TO_DEG * atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                                            q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);

    return &m_mpu_result;
}

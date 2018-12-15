#ifndef _ORIENTATION_CALCULATOR_H_
#define _ORIENTATION_CALCULATOR_H_

#include <stdint.h>

typedef struct
{
    float yaw;
    float pitch;
    float roll;
} orientation_result_t;

const orientation_result_t * calculate_orientation(const float * q); //q must be size for array

#endif // _ORIENTATION_CALCULATOR_H_

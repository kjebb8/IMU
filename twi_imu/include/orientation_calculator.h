#ifndef _ORIENTATION_CALCULATOR_H_
#define _ORIENTATION_CALCULATOR_H_

#include <stdint.h>

typedef union
{
    float    float_val;
    uint8_t  bytes[sizeof(float)];
} float_byte_t;

typedef struct
{
    float_byte_t yaw;
    float_byte_t pitch;
    float_byte_t roll;
} orientation_result_t;

const orientation_result_t * calculate_orientation(const float * q); //q must be size for array

#endif // _ORIENTATION_CALCULATOR_H_

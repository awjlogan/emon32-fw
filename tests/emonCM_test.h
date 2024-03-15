#pragma once

#include <math.h>

/* Redefine the QFPLIB functions to use native floats */
float qfp_fadd(float a, float b)
{
    return a + b;
}

float qfp_fcos(float a)
{
    return (float)cos((double)a);
}

float qfp_fdiv(float a, float b)
{
    return a / b;
}

float qfp_fmul(float a, float b)
{
    return a * b;
}

float qfp_fsin(float a)
{
    return (float)sin((double)a);
}

float qfp_fsub(float a, float b)
{
    return a - b;
}


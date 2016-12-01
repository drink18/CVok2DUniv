#pragma once

const float CV_PI = 3.1415926535f;
const float CV_FLOAT_EPS = 1e-6f;

inline bool cvIsValid(float x)
{
	__int32 ix = *reinterpret_cast<__int32*>(&x);
	return (ix & 0x7f800000) != 0x7f800000;
}

#define DEG2RAD(x) (x * CV_PI / 180.0f)
#define RAD2DEB(x) (x * 180.0f / CV_PI)

#pragma once
#include <cstdint>

const float CV_PI = 3.1415926535f;
const float CV_FLOAT_EPS = 1e-4f;

inline bool cvIsValid(float x)
{
	std::int32_t ix = *reinterpret_cast<std::int32_t*>(&x);
	return (ix & 0x7f800000) != 0x7f800000;
}

#define DEG2RAD(x) (x * CV_PI / 180.0f)
#define RAD2DEB(x) (x * 180.0f / CV_PI)

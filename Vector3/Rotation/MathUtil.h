/////////////////////////////////////////////////////////////////////
//
//   MathUtil.h --- Declarations for miscellaneous math utilities
//
/////////////////////////////////////////////////////////////////////
#ifndef _MATHUTIL_H__
#define _MATHUTIL_H__
#include<math.h>

// 定义和pi有关的常量
const float kPi			= 3.141592657f;
const float k2Pi		= 2.0f * kPi;
const float kPiOver2	= kPi / 2.0f;
const float k1OverPi	= 1.0f / kPi;
const float k1Over2Pi	= 1.0f / k2Pi;

// 通过加适当的2Pi倍数，将角度限制在-Pi和Pi之间
extern float wrapPi(float theta);

// 安全反三角函数
extern float safeAcos(float x);

// 某些平台上，如果需要这两个值，同时计算要比分开计算快
inline void sinCos(float *returnSin, float *returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}


#endif

/////////////////////////////////////////////////////////////////////
//
//   MathUtil.h --- Declarations for miscellaneous math utilities
//
/////////////////////////////////////////////////////////////////////
#ifndef _MATHUTIL_H__
#define _MATHUTIL_H__
#include<math.h>

// �����pi�йصĳ���
const float kPi			= 3.141592657f;
const float k2Pi		= 2.0f * kPi;
const float kPiOver2	= kPi / 2.0f;
const float k1OverPi	= 1.0f / kPi;
const float k1Over2Pi	= 1.0f / k2Pi;

// ͨ�����ʵ���2Pi���������Ƕ�������-Pi��Pi֮��
extern float wrapPi(float theta);

// ��ȫ�����Ǻ���
extern float safeAcos(float x);

// ĳЩƽ̨�ϣ������Ҫ������ֵ��ͬʱ����Ҫ�ȷֿ������
inline void sinCos(float *returnSin, float *returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}


#endif

//////////////////////////////////////////////////////////////////////////
//
//	Quaternion.cpp --- Declarations for class Quaternion
//	w:cos(theta/2) x:nx*sin(theta/2) y:ny*sin(theta/2) z:nz*sin(theta/2)
//
//////////////////////////////////////////////////////////////////////////
#include<assert.h>
#include"MathUtil.h"
#include"Quaternion.h"
#include"Vector3.h"
#include "EulerAngles.h"

const Quaternion kQuaternionIdentity = { 1.0f, 0.0f, 0.0f, 0.0f };

void Quaternion::setToRotateAboutX(float theta)
{
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = sin(thetaOver2);
	y = 0.0f;
	z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta)
{
	float thetaOver2 = theta * 0.5f;
	w = cos(thetaOver2);
	x = 0.0f;
	y = sin(thetaOver2);
	z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta)
{
	float thetaOver2 = theta * 0.5f;
	x = 0.0f;
	y = 0.0f;
	z = sin(thetaOver2);
}

void Quaternion::setToRotateAboutAxis(const Vector3& axis, float theta)
{
	assert(fabs(vectorMag(axis) - 1.0f) < 0.01f);
	float thetaOver2 = theta * 0.5f;
	float sinThetaOver2 = sin(thetaOver2);
	float cosThetaOver2 = cos(thetaOver2);

	w = cosThetaOver2;
	x = axis.x*sinThetaOver2;
	y = axis.y*sinThetaOver2;
	z = axis.z*sinThetaOver2;
}

void Quaternion::setToRotateObjectToInertial(const EulerAngles& orientation)
{
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch * 0.5f);
	sinCos(&sb, &cb, orientation.bank  * 0.5f);
	sinCos(&sh, &ch, orientation.heading * 0.5f);

	w = ch*cp*cb + sh*sp*sb;
	x = ch*sp*cb + sh*cp*sb;
	y = -ch*sp*sb + sh*cp*cb;
	z = -sh*sp*cb + ch*cp*sb;
}

void Quaternion::setToRotateInertialToObject(const EulerAngles& orientation)
{
	float sp, sb, sh;
	float cp, cb, ch;
	sinCos(&sp, &cp, orientation.pitch * 0.5f);
	sinCos(&sb, &cb, orientation.bank  * 0.5f);
	sinCos(&sh, &ch, orientation.heading * 0.5f);

	w = ch*cp*cb + sh*sp*sb;
	x = -ch*sp*cb - sh*cp*sb;
	y = ch*sp*sb - sh*cp*cb;
	z = sh*sp*cb - ch*cp*sb;
}

// 叉乘
Quaternion Quaternion::operator *(const Quaternion& e)const
{
	Quaternion result;
	result.w = w*e.w - x*e.x - y*e.y - z*e.z;
	result.x = w*e.x + x*e.w + z*e.y - y*e.z;
	result.y = w*e.y + y*e.w + x*e.z - z*e.x;
	result.x = w*e.z + z*e.w + y*e.x - x*e.y;
	return result;
} 

Quaternion& Quaternion::operator *=(const Quaternion& e)
{
	*this = *this*e;
	return *this;
}

void Quaternion::normalize()
{
	float mag = (float)sqrt(w*w + x*x + y*y + z*z);
	if (mag > 0.0f)
	{
		float oneOverMag = 1.0f / mag;
		w *= oneOverMag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
	else
	{
		assert(false);
		identity();
	}
}

// 提取旋转角和旋转轴
float Quaternion::getRotateAngle()const
{
	float thetaOver2 = safeAcos(w);
	return 2.0f*thetaOver2;
}

Vector3 Quaternion::getRotateAxis()const
{
	float sinThetaOver2Sq = 1.0f - w*w;
	if (sinThetaOver2Sq<=0.0f)
	{
		return Vector3(1.0f, 0.0f, 0.0f);
	}
	float oneOverSinThetaOver2 = 1.0f / sqrt(sinThetaOver2Sq);
	return Vector3(x*oneOverSinThetaOver2, y*oneOverSinThetaOver2, z*oneOverSinThetaOver2);
}

/////////////////////////////////////////////////////////////////////
//
//							非成员函数
//
/////////////////////////////////////////////////////////////////////

// 四元数点乘
float dotProduct(const Quaternion& a, const Quaternion&b)
{
	return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

// 球面线性插值
Quaternion slerp(const Quaternion& a, const Quaternion& b, float t)
{
	if (t <= 0.0f) return a;
	if (t >= 1.0f) return b;

	float cosOmega = dotProduct(a, b);
	float bw = b.w,
		  bx = b.x,
		  by = b.y,
		  bz = b.z;
	if (cosOmega<0.0f)
	{
		bw = -bw;
		bx = -bx;
		by = -by;
		bz = -bz;
		cosOmega = -cosOmega;
	}
	assert(cosOmega < 1.1f);
	float k0, k1;
	if (cosOmega > 0.999f)
	{
		k0 = 1.0f - t;
		k1 = t;
	}
	else
	{
		float sinOmega = sqrt(1.0f - cosOmega*cosOmega);
		float omega = atan2(sinOmega, cosOmega);
		float oneOverSinOmega = 1.0f / sinOmega;

		k0 = sin((1.0f - t)*omega)*oneOverSinOmega;
		k1 = sin(t*omega)*oneOverSinOmega;
	}
	Quaternion result;
	result.w = k0*a.w + k1*bw;
	result.x = k0*a.x + k1*bx;
	result.y = k0*a.y + k1*by;
	result.z = k0*a.z + k1*bz;
}

// 四元数共轭
Quaternion conjugate(const Quaternion& e)
{
	Quaternion result;
	result.w = e.w;
	result.x = -e.x;
	result.y = -e.y;
	result.z = -e.z;
	return result;
}

// 四元数幂
Quaternion pow(const Quaternion& e, float exponent)
{
	if (fabs(e.w)>0.999f)
	{
		return e;
	}

	float alpha = acos(e.w);
	float newAlpha = exponent*alpha;

	Quaternion result;
	result.w = cos(newAlpha);
	
	float mult = sin(newAlpha) / sin(alpha);
	result.x = e.x*mult;
	result.y = e.y*mult;
	result.z = e.z*mult;
	return result;
}
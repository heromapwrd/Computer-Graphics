////////////////////////////////////////////////////////////
//
//					Vector3.h
//
////////////////////////////////////////////////////////////
#include "Vector3.h"
#include<math.h>
Vector3::Vector3()
{

}

Vector3::Vector3(const Vector3& v) :x(v.x), y(v.y), z(v.z)
{

}

Vector3::Vector3(float fx, float fy, float fz) : x(fx), y(fy), z(fz)
{

}

Vector3& Vector3::operator =(const Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z; 
	return *this;
}

bool Vector3::operator == (const Vector3& v)const
{
	return (x == v.x) && (y == v.y) && (z == v.z);
}

bool Vector3::operator != (const Vector3& v)const
{
	return (x != v.x) || (y != v.y) || (z != v.z);
}

Vector3 Vector3::operator +(const Vector3& v)const
{
	return Vector3(x + v.x, y + v.y, z + v.z);
}

Vector3 Vector3::operator -(const Vector3& v)const
{
	return Vector3(x - v.x, y - v.y, z - v.z);
}

Vector3 Vector3::operator *(float a)const
{
	return Vector3(x*a, y*a, z*a);
}

Vector3 Vector3::operator /(float a)const
{
	float oneOverA = 1 / a;
	return Vector3(x*oneOverA, y*oneOverA, z*oneOverA);
}

Vector3& Vector3::operator +=(const Vector3& v)
{
	x += v.x;
	y += v.y;
	z += v.z;
	return *this;
}

Vector3& Vector3::operator -=(const Vector3& v)
{
	x -= v.x;
	y -= v.y;
	z -= v.z;
	return *this;
}

Vector3& Vector3::operator *=(float a)
{
	x *= a;
	y *= a;
	z *= a;
	return *this;
}

Vector3& Vector3::operator /=(float a)
{
	x /= a;
	y /= a;
	z /= a;
	return *this;
}

void Vector3::normalize()
{
	float magSq = x*x + y*y + z*z;
	if (magSq > 0.0f)
	{
		float oneOverMag = 1.0f / sqrt(magSq);
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
	}
}

float Vector3::operator * (const Vector3& v)const
{
	return x*v.x + y*v.y + z*v.z;
}
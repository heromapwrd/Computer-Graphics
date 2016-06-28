////////////////////////////////////////////////////////////
//
//					Vector3.h
//
////////////////////////////////////////////////////////////
#ifndef _VECTOR3_H__
#define _VECTOR#_H__
class Vector3
{
public:
	Vector3();
	Vector3(const Vector3& v);
	Vector3(float fx, float fy, float fz);
	Vector3& operator =(const Vector3& v);
	bool operator == (const Vector3& v)const;
	bool operator != (const Vector3& v)const;
	void Zero(){ x = y = z = 0.0f; }
	Vector3 operator - ()const { return Vector3(-x, -y, -z); }
	Vector3 operator +(const Vector3& v)const;
	Vector3 operator -(const Vector3& v)const;
	Vector3 operator *(float a)const;
	Vector3 operator /(float a)const;
	Vector3& operator +=(const Vector3& v);
	Vector3& operator -=(const Vector3& v);
	Vector3& operator *=(float a);
	Vector3& operator /=(float a);
	void normalize();
	float operator * (const Vector3& v)const;
public:
	float x, y, z;
};

inline float vectorMag(const Vector3& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

inline Vector3 crossProduct(const Vector3& a, const Vector3& b)
{
	return Vector3(a.y*b.z - a.z*b.y,
		a.z*b.x - a.x*b.z,
		a.x*b.y - a.y*b.x);
}

inline Vector3 operator* (float k, const Vector3& v)
{
	return Vector3(k*v.x, k*v.y, k*v.z);
}

inline float distance(const Vector3& a, const Vector3& b)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}


//////////////////////////////////////////////////////////////
//
//			Global Variables
//
//////////////////////////////////////////////////////////////
extern const Vector3 kZeroVector;

#endif
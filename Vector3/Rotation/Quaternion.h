//////////////////////////////////////////////////////////////////////////
//
//	Quaternion.h --- Declarations for class Quaternion
//	w:cos(theta/2) x:nx*sin(theta/2) y:ny*sin(theta/2) z:nz*sin(theta/2)
//
//////////////////////////////////////////////////////////////////////////
#ifndef _QUATERNION_H__
#define _QUATERNION_H__
class Vector3;
class EulerAngles;

class Quaternion
{
public:
	float w, x, y, z;
public:
	Quaternion(){}
	Quaternion(float fw, float fx, float fy, float fz) :w(fw), x(fx), y(fy), z(fz)
	{

	}
	void identity(){ w = 1.0f, x = y = z = 0.0f; }
	void setToRotateAboutX(float theta);
	void setToRotateAboutY(float theta);
	void setToRotateAboutZ(float theta);
	void setToRotateAboutAxis(const Vector3& axis,float theta);

	void setToRotateObjectToInertial(const EulerAngles& orientation);
	void setToRotateInertialToObject(const EulerAngles& orientation);

	// 叉乘
	Quaternion operator *(const Quaternion& e)const;
	Quaternion& operator *=(const Quaternion& e);

	void normalize();

	// 提取旋转角和旋转轴
	float getRotateAngle()const;
	Vector3 getRotateAxis()const;
};

// 全局单位四元数
extern const Quaternion kQuaternionIdentity;
// 四元数点乘
extern float dotProduct(const Quaternion& a, const Quaternion&b);
// 球面线性插值
extern Quaternion slerp(const Quaternion& a, const Quaternion& b, float t);
// 四元数共轭
extern Quaternion conjugate(const Quaternion& e);
// 四元数幂
extern Quaternion pow(const Quaternion& e, float exponent);

#endif
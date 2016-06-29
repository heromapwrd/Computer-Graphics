////////////////////////////////////////////////////////////////
//
//	RotationMatrix.h --- Declarations for class RotationMatrix
//	旋转轴:(nx,ny,nz)  角度:theta
//
////////////////////////////////////////////////////////////////
#ifndef _ROTATIONMATRIX_H__
#define _ROTATIONMATRIX_H__
class Vector3;
class EulerAngles;
class Quaternion;

class RotationMatrix
{
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
public:
	void identity();
	// 根据欧拉角构造矩阵
	void setup(const EulerAngles& oritention);
	// 根据四元数构造矩阵
	void fromInertialToObjectQuaternion(const Quaternion& q);
	void fromObjectToInertialQuaternion(const Quaternion& q);

	// 执行旋转
	Vector3 inertialToObject(const Vector3& v);
	Vector3 objectToInertial(const Vector3& v);
};



#endif
////////////////////////////////////////////////////////////////
//
//	RotationMatrix.h --- Declarations for class RotationMatrix
//	��ת��:(nx,ny,nz)  �Ƕ�:theta
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
	// ����ŷ���ǹ������
	void setup(const EulerAngles& oritention);
	// ������Ԫ���������
	void fromInertialToObjectQuaternion(const Quaternion& q);
	void fromObjectToInertialQuaternion(const Quaternion& q);

	// ִ����ת
	Vector3 inertialToObject(const Vector3& v);
	Vector3 objectToInertial(const Vector3& v);
};



#endif
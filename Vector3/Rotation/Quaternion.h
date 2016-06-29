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

	// ���
	Quaternion operator *(const Quaternion& e)const;
	Quaternion& operator *=(const Quaternion& e);

	void normalize();

	// ��ȡ��ת�Ǻ���ת��
	float getRotateAngle()const;
	Vector3 getRotateAxis()const;
};

// ȫ�ֵ�λ��Ԫ��
extern const Quaternion kQuaternionIdentity;
// ��Ԫ�����
extern float dotProduct(const Quaternion& a, const Quaternion&b);
// �������Բ�ֵ
extern Quaternion slerp(const Quaternion& a, const Quaternion& b, float t);
// ��Ԫ������
extern Quaternion conjugate(const Quaternion& e);
// ��Ԫ����
extern Quaternion pow(const Quaternion& e, float exponent);

#endif
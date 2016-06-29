//////////////////////////////////////////////////////////////
//
//	Matrix4x3.h --- Declarations for class Matrix4x3
//	旋转轴:(nx,ny,nz)  角度:theta
//
//////////////////////////////////////////////////////////////
#ifndef _MATRIX4X3_H__
#define _MATRIX4X3_H__
class Vector3;
class EulerAngles;
class Quaternion;
class RotationMatrix;

class Matrix4x3
{
public:
	float m11, m12, m13;
	float m21, m22, m23;
	float m31, m32, m33;
	float tx, ty, tz;
public:
	void identity();
	// 访问平移部分
	void zeroTranslation();
	void setTranslation(const Vector3& v);
	void setupTranslation(const Vector3& v);

	// 后续

};

#endif
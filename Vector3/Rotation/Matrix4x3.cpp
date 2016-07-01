//////////////////////////////////////////////////////////////
//
//	Matrix4x3.h --- Declarations for class Matrix4x3
//	��ת��:(nx,ny,nz)  �Ƕ�:theta
//
//////////////////////////////////////////////////////////////
#include<assert.h>
#include "Vector3.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include "RotationMatrix.h"
#include "MathUtil.h"
#include "Matrix4x3.h"

void Matrix4x3::identity()
{
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = 0.0f; ty = 0.0f; tz = 1.0f;
}

// ����ƽ�Ʋ���
void Matrix4x3::zeroTranslation()
{
	tx = ty = tz = 0.0f;
}

void Matrix4x3::setTranslation(const Vector3& v)
{
	tx = v.x;
	ty = v.y;
	tz = v.z;
}

void Matrix4x3::setupTranslation(const Vector3& v)
{
	m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
	tx = v.x; ty = v.y; tz = v.z;
}

// ����ִ�и��ռ�<->�ֲ��ռ�任�ľ��󣬼ٶ��ֲ��ռ���ָ����λ�úͷ��򣬸�λ������ʹ��ŷ���ǻ�����ת�����ʾ
void Matrix4x3::setupLocalToParent(const Vector3& pos, const EulerAngles& orient)
{
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);

	setupLocalToParent(pos, orientMatrix);
}

void Matrix4x3::setupLocalToParent(const Vector3& pos, const RotationMatrix& orient)
{
	m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
	m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
	m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;
	tx = pos.x; ty = pos.y; tz = pos.z;
}

void Matrix4x3::setupParentToLocal(const Vector3& pos, const EulerAngles& orient)
{
	RotationMatrix orientMatrix;
	orientMatrix.setup(orient);
	setupParentToLocal(pos, orientMatrix);
}

void Matrix4x3::setupParentToLocal(const Vector3& pos, const RotationMatrix& orient)
{
	m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
	m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
	m31 = orient.m31; m32 = orient.m32; m33 = orient.m33;
	tx = -(pos.x*m11 + pos.y*m21 + pos.z*m31);
	ty = -(pos.x*m12 + pos.y*m22 + pos.z*m32);
	tz = -(pos.x*m13 + pos.y*m23 + pos.z*m33);
}

// ��������������ת�ľ���
void Matrix4x3::setupRotate(int axis, float theta)
{
	float s, c;
	sinCos(&s, &c, theta);

	switch (axis)
	{
	case 1:
		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = c; m23 = s;
		m31 = 0.0f; m32 = -s; m33 = c;
		break;
	case 2:
		m11 = c; m12 = 0.0f; m13 = -s;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s; m32 = 0.0f; m33 = c;
		break;
	case 3:
		m11 = c; m12 = s; m13 = 0.0f;
		m21 = -s; m22 = c; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;
	default:
		assert(false);
	}
	tx = ty = tz = 0.0f;
}

// ����������������ת�ľ���
void Matrix4x3::setupRotate(const Vector3& axis, float theta)
{
	assert(fabs(axis*axis - 1.0f) < .01f);

	float	s, c;
	sinCos(&s, &c, theta);

	float	a = 1.0f - c;
	float	ax = a * axis.x;
	float	ay = a * axis.y;
	float	az = a * axis.z;

	m11 = ax*axis.x + c;
	m12 = ax*axis.y + axis.z*s;
	m13 = ax*axis.z - axis.y*s;

	m21 = ay*axis.x - axis.z*s;
	m22 = ay*axis.y + c;
	m23 = ay*axis.z + axis.x*s;

	m31 = az*axis.x + axis.y*s;
	m32 = az*axis.y - axis.x*s;
	m33 = az*axis.z + c;

	tx = ty = tz = 0.0f;
}

// ������ת���󣬽�λ������Ԫ����ʽ����
void  Matrix4x3::fromQuaternion(const Quaternion& q)
{
	float	ww = 2.0f * q.w;
	float	xx = 2.0f * q.x;
	float	yy = 2.0f * q.y;
	float	zz = 2.0f * q.z;

	m11 = 1.0f - yy*q.y - zz*q.z;
	m12 = xx*q.y + ww*q.z;
	m13 = xx*q.z - ww*q.x;

	m21 = xx*q.y - ww*q.z;
	m22 = 1.0f - xx*q.x - zz*q.z;
	m23 = yy*q.z + ww*q.x;

	m31 = xx*q.z + ww*q.y;
	m32 = yy*q.z - ww*q.x;
	m33 = 1.0f - xx*q.x - yy*q.y;

	tx = ty = tz = 0.0f;

}

// ���������������ŵľ���
void Matrix4x3::setupScale(const Vector3& s)
{
	m11 = s.x;  m12 = 0.0f; m13 = 0.0f;
	m21 = 0.0f; m22 = s.y;  m23 = 0.0f;
	m31 = 0.0f; m32 = 0.0f; m33 = s.z;

	tx = ty = tz = 0.0f;
}

// ���������������ŵľ���
void Matrix4x3::SetupScaleAlongAxis(const Vector3& axis, float k)
{
	assert(fabs(axis*axis - 1.0f) < .01f);

	float	a = k - 1.0f;
	float	ax = a * axis.x;
	float	ay = a * axis.y;
	float	az = a * axis.z;

	m11 = ax*axis.x + 1.0f;
	m22 = ay*axis.y + 1.0f;
	m32 = az*axis.z + 1.0f;

	m12 = m21 = ax*axis.y;
	m13 = m31 = ax*axis.z;
	m23 = m32 = ay*axis.z;

	tx = ty = tz = 0.0f;

}

// �����б߾���
void Matrix4x3::setupShear(int axis, float s, float t)
{
	switch (axis) {

	case 1: // Shear y and z using x

		m11 = 1.0f; m12 = s;    m13 = t;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;

	case 2: // Shear x and z using y

		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = s;    m22 = 1.0f; m23 = t;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
		break;

	case 3: // Shear x and y using z

		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = s;    m32 = t;    m33 = 1.0f;
		break;

	default:
		assert(false);
	}

	tx = ty = tz = 0.0f;

}

// ����ͶӰ����ͶӰƽ���ԭ��
void Matrix4x3::setupProject(const Vector3& n)
{
	assert(fabs(n*n - 1.0f) < .01f);

	m11 = 1.0f - n.x*n.x;
	m22 = 1.0f - n.y*n.y;
	m33 = 1.0f - n.z*n.z;

	m12 = m21 = -n.x*n.y;
	m13 = m31 = -n.x*n.z;
	m23 = m32 = -n.y*n.z;

	tx = ty = tz = 0.0f;

}

// ���췴�����
void Matrix4x3::setupReflect(int axis, float k = 0.0f)
{
	switch (axis) {

	case 1: // Reflect about the plane x=k

		m11 = -1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		tx = 2.0f * k;
		ty = 0.0f;
		tz = 0.0f;

		break;

	case 2: // Reflect about the plane y=k

		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = -1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;

		tx = 0.0f;
		ty = 2.0f * k;
		tz = 0.0f;

		break;

	case 3: // Reflect about the plane z=k

		m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
		m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
		m31 = 0.0f; m32 = 0.0f; m33 = -1.0f;

		tx = 0.0f;
		ty = 0.0f;
		tz = 2.0f * k;

		break;

	default:
		assert(false);
	}

}

// ����������ƽ�淴��ľ���
void Matrix4x3::setupReflect(const Vector3& n)
{
	assert(fabs(n*n - 1.0f) < .01f);

	float	ax = -2.0f * n.x;
	float	ay = -2.0f * n.y;
	float	az = -2.0f * n.z;

	m11 = 1.0f + ax*n.x;
	m22 = 1.0f + ay*n.y;
	m32 = 1.0f + az*n.z;

	m12 = m21 = ax*n.y;
	m13 = m31 = ax*n.z;
	m23 = m32 = ay*n.z;

	tx = ty = tz = 0.0f;

}


////////////////////////////////////////////////////////////////
//
//				�ǳ�Ա����
//
////////////////////////////////////////////////////////////////

// �����*�����任������Ӿ��󣬳˷���˳����������ر任��˳�����
Vector3 operator*(const Vector3& p, const Matrix4x3& m)
{

}

Matrix4x3 operator*(const Matrix4x3& a, const Matrix4x3& b)
{

}

// �����*=�����ֺ�C++��׼�﷨��һ����
Vector3& operator*=(const Vector3& p, const Matrix4x3& m)
{

}

Matrix4x3& operator*=(const Matrix4x3& a, const Matrix4x3& b)
{

}

// ����3x3���ֵ�����ʽֵ
float determinant(const Matrix4x3& m)
{

}

// ����������
Matrix4x3 inverse(const Matrix4x3& m)
{

}

// ��ȡ�����ƽ�Ʋ���
Vector3 getTranslation(const Matrix4x3& m)
{

}

// �Ӿֲ��������������߸�������ֲ�����ȡλ��/��λ
Vector3 getPositionFromParentToLocalMatrix(const Matrix4x3& m)
{

}

Vector3 getPositionFromLocalToParentMatrix(const Matrix4x3& m)
{

}
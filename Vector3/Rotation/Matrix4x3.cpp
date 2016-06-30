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

void identity()
{

}

// ����ƽ�Ʋ���
void Matrix4x3::zeroTranslation()
{

}

void Matrix4x3::setTranslation(const Vector3& v)
{

}

void Matrix4x3::setupTranslation(const Vector3& v)
{

}

// ����ִ�и��ռ�<->�ֲ��ռ�任�ľ��󣬼ٶ��ֲ��ռ���ָ����λ�úͷ��򣬸�λ������ʹ��ŷ���ǻ�����ת�����ʾ
void Matrix4x3::setupLocalToParent(const Vector3& pos, const EulerAngles& orient)
{

}

void Matrix4x3::setupLocalToParent(const Vector3& pos, const RotationMatrix& orient)
{

}

void Matrix4x3::setupParentToLocal(const Vector3& pos, const EulerAngles& orient)
{

}

void Matrix4x3::setupParentToLocal(const Vector3& pos, const RotationMatrix& orient)
{

}

// ��������������ת�ľ���
void Matrix4x3::setupRotate(int axis, float theta)
{

}

// ����������������ת�ľ���
void Matrix4x3::setupRotate(const Vector3& axis, float theta)
{

}

// ������ת���󣬽�λ������Ԫ����ʽ����
void  Matrix4x3::fromQuaternion(const Quaternion& q)
{

}

// ���������������ŵľ���
void Matrix4x3::setupScale(const Vector3& s)
{

}

// ���������������ŵľ���
void Matrix4x3::SetupScale(const Vector3& axis, float k)
{

}

// �����б߾���
void Matrix4x3::setupShear(int aixs, float s, float t)
{

}

// ����ͶӰ����ͶӰƽ���ԭ��
void Matrix4x3::setupProject(const Vector3& n)
{

}

// ���췴�����
void Matrix4x3::setupReflect(int axis, float k = 0.0f)
{

}

// ����������ƽ�淴��ľ���
void Matrix4x3::setupReflect(const Vector3& n)
{

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
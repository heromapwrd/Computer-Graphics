//////////////////////////////////////////////////////////////
//
//	EulerAngles.cpp --- Declarations for class EulerAngles
//	用于表示 Heanding-Pitch-Bank角
//
//////////////////////////////////////////////////////////////
#include "MathUtil.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

// 变换为“限制集”欧拉角
void EulerAngles::cononize()
{
	pitch = wrapPi(pitch);
	if (pitch < -kPiOver2)
	{
		pitch = -kPi - pitch;
		heading += kPi;
		bank += kPi;
	}
	else if (pitch>kPiOver2)
	{
		pitch = kPi - pitch;
		heading += kPi;
		bank += kPi;
	}

	if (fabs(pitch) > kPiOver2 - 1e-4)
	{
		heading += bank;
		bank = 0.0f;
	}
	else
	{
		bank = wrapPi(bank);
	}
	heading = wrapPi(heading);
}

// 从四元数转换到欧拉角
// 输入的丝源叔假设为物体-惯性或者惯性-物体四元数，如其名所示
void EulerAngles::fromObjectToInertialQuaternion(const Quaternion& q)
{
	// 计算sin(pitch)
	float sp = -2.0f*(q.y*q.z - q.w*q.x);
	// 检查万向锁
	if (fabs(sp) > 0.999f)
	{
		pitch = kPiOver2*sp;
		heading = atan2(-q.x*q.z + q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;
	}
	else
	{
		pitch = asin(sp);
		heading = atan2(q.x*q.z + q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}

void EulerAngles::fromInertialToObjectQuaternion(const Quaternion& q)
{
	// 计算sin(pitch)
	float sp = -2.0f*(q.y*q.z + q.w*q.x);
	// 检查万向锁
	if (fabs(sp) > 0.999f)
	{
		pitch = kPiOver2*sp;
		heading = atan2(-q.x*q.z - q.w*q.y, 0.5f - q.y*q.y - q.z*q.z);
		bank = 0.0f;
	}
	else
	{
		pitch = asin(sp);
		heading = atan2(q.x*q.z - q.w*q.y, 0.5f - q.x*q.x - q.y*q.y);
		bank = atan2(q.x*q.y + q.w*q.z, 0.5f - q.x*q.x - q.z*q.z);
	}
}

// 从矩阵转换到欧拉角
// 输入矩阵假设为物体-世界或者世界-物体转换矩阵
// 评议部分被省略，并且假设矩阵式正交的
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3& m)
{
	// 根据m32计算sin(pitch)
	float sp = -m.m32;
	// 检查万向锁
	if (fabs(sp) > 0.99999f)
	{
		pitch = kPiOver2*sp;
		heading = atan2(-m.m13, m.m11);
		bank = 0.0f;
	}
	else
	{
		heading = atan2(m.m31, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m12, m.m22);
	}
}

void EulerAngles::fromWorldToObjectMatrix(const Matrix4x3& m)
{
	// 根据m23计算sin(pitch)
	float sp = -m.m23;
	// 检查万向锁
	if (fabs(sp)>0.99999f)
	{
		pitch = kPiOver2*sp;
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else
	{
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}

// 从旋转矩阵到欧拉角
void EulerAngles::fromRotationMatrix(const RotationMatrix& m)
{
	// 根据m23计算sin(pitch)
	float sp = -m.m23;
	// 检查万向锁
	if (fabs(sp) > 0.99999f)
	{
		pitch = kPiOver2*sp;
		heading = atan2(-m.m31, m.m11);
		bank = 0.0f;
	}
	else
	{
		heading = atan2(m.m13, m.m33);
		pitch = asin(sp);
		bank = atan2(m.m21, m.m22);
	}
}

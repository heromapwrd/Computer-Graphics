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

}

void EulerAngles::fromInertialToObjectQuaternion(const Quaternion& q)
{

}

// 从矩阵转换到欧拉角
// 输入矩阵假设为物体-世界或者世界-物体转换矩阵
// 评议部分被省略，并且假设矩阵式正交的
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3& m)
{

}

void EulerAngles::fromWorldToobjectMatrix(const Matrix4x3& m)
{

}

// 从旋转矩阵到欧拉角
void EulerAngles::fromRotationMatrix(const RotationMatrix& m)
{

}

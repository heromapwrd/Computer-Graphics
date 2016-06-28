//////////////////////////////////////////////////////////////
//
//	EulerAngles.cpp --- Declarations for class EulerAngles
//	���ڱ�ʾ Heanding-Pitch-Bank��
//
//////////////////////////////////////////////////////////////
#include "MathUtil.h"
#include "EulerAngles.h"
#include "Quaternion.h"
#include "Matrix4x3.h"
#include "RotationMatrix.h"

// �任Ϊ�����Ƽ���ŷ����
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

// ����Ԫ��ת����ŷ����
// �����˿Դ�����Ϊ����-���Ի��߹���-������Ԫ������������ʾ
void EulerAngles::fromObjectToInertialQuaternion(const Quaternion& q)
{

}

void EulerAngles::fromInertialToObjectQuaternion(const Quaternion& q)
{

}

// �Ӿ���ת����ŷ����
// ����������Ϊ����-�����������-����ת������
// ���鲿�ֱ�ʡ�ԣ����Ҽ������ʽ������
void EulerAngles::fromObjectToWorldMatrix(const Matrix4x3& m)
{

}

void EulerAngles::fromWorldToobjectMatrix(const Matrix4x3& m)
{

}

// ����ת����ŷ����
void EulerAngles::fromRotationMatrix(const RotationMatrix& m)
{

}

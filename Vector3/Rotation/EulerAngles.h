//////////////////////////////////////////////////////////////
//
//	EulerAngles.h --- Declarations for class EulerAngles
//	���ڱ�ʾ Heanding-Pitch-Bank��
//
//////////////////////////////////////////////////////////////
#ifndef _EULERANGLES_H__
#define _EULERANGLES_H__

class Quaternion;
class Matrix4x3;
class RotationMatrix;

class EulerAngles
{
public:
	float heading;
	float pitch;
	float bank;
public:
	EulerAngles(){}
	EulerAngles(float h, float p, float b) :heading(h), pitch(p), bank(b)
	{

	}
	EulerAngles(const EulerAngles& e) :heading(e.heading), pitch(e.pitch), bank(e.bank)
	{

	}

	// ����
	void identity(){ heading = pitch = bank = 0.0f; }
	// �任Ϊ�����Ƽ���ŷ����
	void cononize();
	// ����Ԫ��ת����ŷ����
	// �����˿Դ�����Ϊ����-���Ի��߹���-������Ԫ������������ʾ
	void fromObjectToInertialQuaternion(const Quaternion& q);
	void fromInertialToObjectQuaternion(const Quaternion& q);
	// �Ӿ���ת����ŷ����
	// ����������Ϊ����-�����������-����ת������
	// ���鲿�ֱ�ʡ�ԣ����Ҽ������ʽ������
	void fromObjectToWorldMatrix(const Matrix4x3& m);
	void fromWorldToObjectMatrix(const Matrix4x3& m);
	// ����ת����ŷ����
	void fromRotationMatrix(const RotationMatrix& m);
};

// ȫ�֡���λ��ŷ����
extern const EulerAngles kEulerAnglesIndentity;




#endif

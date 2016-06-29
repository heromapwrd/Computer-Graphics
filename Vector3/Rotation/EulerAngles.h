//////////////////////////////////////////////////////////////
//
//	EulerAngles.h --- Declarations for class EulerAngles
//	用于表示 Heanding-Pitch-Bank角
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

	// 置零
	void identity(){ heading = pitch = bank = 0.0f; }
	// 变换为“限制集”欧拉角
	void cononize();
	// 从四元数转换到欧拉角
	// 输入的丝源叔假设为物体-惯性或者惯性-物体四元数，如其名所示
	void fromObjectToInertialQuaternion(const Quaternion& q);
	void fromInertialToObjectQuaternion(const Quaternion& q);
	// 从矩阵转换到欧拉角
	// 输入矩阵假设为物体-世界或者世界-物体转换矩阵
	// 评议部分被省略，并且假设矩阵式正交的
	void fromObjectToWorldMatrix(const Matrix4x3& m);
	void fromWorldToObjectMatrix(const Matrix4x3& m);
	// 从旋转矩阵到欧拉角
	void fromRotationMatrix(const RotationMatrix& m);
};

// 全局“单位”欧拉角
extern const EulerAngles kEulerAnglesIndentity;




#endif

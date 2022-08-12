#include "Utils.h"

namespace orangelie
{
	// ��ü Ŭ����
	class RigidBody
	{
	public:
		// ���µ����͸� ���� ���ε����͸� ����մϴ�.
		void CalculateDerivedData();

	private:
		// ������ ����
		float mInverseMass;
		// ������� ���Ǵ� ���ξ�
		float mLinearDamping;
		// ȸ����� ���Ǵ� ���ξ�
		float mAngularDamping;
		// ��ü�� ���
		float mMotion;

		// ��ü�� ����������� ������ �����ʵ��� �����մϴ�.
		bool mIsAwake;
		// ��ü�� ����� ���ϵ��� �մϴ�.
		bool mCanSleep;

		// ������������� ��ü�� ��ġ
		DirectX::XMFLOAT3 mPosition;
		// ������������� ��ü�� ����
		DirectX::XMFLOAT4 mOrientation;
		// ������������� ��ü�� ���� �ӵ�
		DirectX::XMFLOAT3 mVelocity;
		// ������������� ��ü�� ȸ�� �ӵ�, �Ǵ� ȸ��
		DirectX::XMFLOAT3 mRotation;
		// ���� ���дܰ迡�� ����� ���� ����
		DirectX::XMFLOAT3 mForceAccum;
		// ���� ���дܰ迡�� ����� ��ũ�� ����
		DirectX::XMFLOAT3 mTorqueAccum;
		// ��ü�� �����ӵ�
		DirectX::XMFLOAT3 mAcceleration;
		// �� �����ӿ����� �����ӵ��� ����
		DirectX::XMFLOAT3 mLastframeAcceleration;

		// ��ü���������� �����ټ��� �����
		DirectX::XMFLOAT3X3 mInverseInertiaTensor;
		// ������������� �����ټ��� �����
		DirectX::XMFLOAT3X3 mInverseInertiaTensorWorld;

		// ��ü������ ����������� �ٲٴ� ��ȯ ���
		DirectX::XMFLOAT4X4 mTransformMatrix;

	};
}

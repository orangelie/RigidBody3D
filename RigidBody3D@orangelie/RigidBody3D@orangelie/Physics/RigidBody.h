#include "Utils.h"

constexpr static const float gSleepEpsilon = 0.3f;

namespace orangelie
{


	// ��ü Ŭ����
	class RigidBody
	{
	public:
		// ���µ����͸� ���� ���ε����͸� ����մϴ�.
		void CalculateDerivedData();
		// ��ü�� �������� �����ŭ �����մϴ�.
		void Integrate(float dt);

		void setAwake(const bool awake);
		void SetMass(const float mass);
		void SetVelocity(const float x, const float y, const float z);
		void SetAcceleration(const float x, const float y, const float z);
		void SetDamping(const float linearDamping, const float angularDamping);

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
		DirectX::XMFLOAT4 mPosition;
		// ������������� ��ü�� ����
		DirectX::XMFLOAT4 mOrientation;
		// ������������� ��ü�� ���� �ӵ�
		DirectX::XMFLOAT4 mVelocity;
		// ������������� ��ü�� ȸ�� �ӵ�, �Ǵ� ȸ��
		DirectX::XMFLOAT4 mRotation;
		// ���� ���дܰ迡�� ����� ���� ����
		DirectX::XMFLOAT4 mForceAccum;
		// ���� ���дܰ迡�� ����� ��ũ�� ����
		DirectX::XMFLOAT4 mTorqueAccum;
		// ��ü�� �����ӵ�
		DirectX::XMFLOAT4 mAcceleration;
		// �� �����ӿ����� �����ӵ��� ����
		DirectX::XMFLOAT4 mLastframeAcceleration;

		// ��ü���������� �����ټ��� �����
		DirectX::XMFLOAT4X4 mInverseInertiaTensor;
		// ������������� �����ټ��� �����
		DirectX::XMFLOAT4X4 mInverseInertiaTensorWorld;

		// ��ü������ ����������� �ٲٴ� ��ȯ ���
		DirectX::XMFLOAT4X4 mTransformMatrix;

	};
}

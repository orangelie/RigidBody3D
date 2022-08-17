#pragma once

#include "Utils.h"


constexpr static const float gSleepEpsilon = 0.3f;

namespace orangelie
{
	namespace Physics
	{


		// ��ü Ŭ����
		class RigidBody
		{
		public:
			// ���µ����͸� ���� ���ε����͸� ����մϴ�.
			void CalculateDerivedData();
			// ��ü�� �������� �����ŭ �����մϴ�.
			void Integrate(float dt);
			void ClearAccmulator();

			void AddForce(float x, float y, float z);
			void SetPosition(const float x, const float y, const float z);
			void setAwake(const bool awake = true);
			void setSleep(const bool sleep);
			void SetMass(const float mass);
			void SetVelocity(const float x, const float y, const float z);
			void SetAcceleration(const float x, const float y, const float z);
			void SetRotation(const float x, const float y, const float z);
			void SetOrientation(const float i, const float j, const float k, const float r);
			void SetDamping(const float linearDamping, const float angularDamping);
			void SetInertiaTensor(const DirectX::XMFLOAT4X4& InertiaTensor);

			void AddVelocity(const float x, const float y, const float z);
			void AddRotation(const float x, const float y, const float z);

			void GetPosition(DirectX::XMFLOAT4& Position);
			void GetRotation(DirectX::XMFLOAT4& Rotation);
			void GetVelocity(DirectX::XMFLOAT4& Velocity);
			float GetInverseMass() const;
			bool GetAwake() const;
			void GetLastframeAcceleration(DirectX::XMFLOAT4& Acceleration);
			void GetTransform(DirectX::XMFLOAT4X4& transformMatrix);
			void GetInverseInertiaTensorWorld(DirectX::XMFLOAT4X4& InertiaTensor);

		private:
			// ������ ����
			float mInverseMass = 0.0f;
			// ������� ���Ǵ� ���ξ�
			float mLinearDamping = 0.0f;
			// ȸ����� ���Ǵ� ���ξ�
			float mAngularDamping = 0.0f;
			// ��ü�� ���
			float mMotion = 0.0f;

			// ��ü�� ����������� ������ �����ʵ��� �����մϴ�.
			bool mIsAwake = false;
			// ��ü�� ����� ���ϵ��� �մϴ�.
			bool mCanSleep = false;

			// ������������� ��ü�� ��ġ
			DirectX::XMFLOAT4 mPosition = {};
			// ������������� ��ü�� ����
			DirectX::XMFLOAT4 mOrientation = {};
			// ������������� ��ü�� ���� �ӵ�
			DirectX::XMFLOAT4 mVelocity = {};
			// ������������� ��ü�� ȸ�� �ӵ�, �Ǵ� ȸ��
			DirectX::XMFLOAT4 mRotation = {};
			// ���� ���дܰ迡�� ����� ���� ����
			DirectX::XMFLOAT4 mForceAccum = {};
			// ���� ���дܰ迡�� ����� ��ũ�� ����
			DirectX::XMFLOAT4 mTorqueAccum = {};
			// ��ü�� �����ӵ�
			DirectX::XMFLOAT4 mAcceleration = {};
			// �� �����ӿ����� �����ӵ��� ����
			DirectX::XMFLOAT4 mLastframeAcceleration = {};

			// ��ü���������� �����ټ��� �����
			DirectX::XMFLOAT4X4 mInverseInertiaTensor = {};
			// ������������� �����ټ��� �����
			DirectX::XMFLOAT4X4 mInverseInertiaTensorWorld = {};
			// ��ü������ ����������� �ٲٴ� ��ȯ ���
			DirectX::XMFLOAT4X4 mTransformMatrix = {};

		};
	}
}

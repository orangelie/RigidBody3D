#pragma once

#include "Utils.h"


constexpr static const float gSleepEpsilon = 0.3f;

namespace orangelie
{
	namespace Physics
	{


		// 강체 클래스
		class RigidBody
		{
		public:
			// 상태데이터를 통해 내부데이터를 계산합니다.
			void CalculateDerivedData();
			// 강체에 가해지는 운동량만큼 적분합니다.
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
			// 질량의 역수
			float mInverseMass = 0.0f;
			// 선형운동에 사용되는 댐핑양
			float mLinearDamping = 0.0f;
			// 회전운동에 사용되는 댐핑양
			float mAngularDamping = 0.0f;
			// 강체의 운동량
			float mMotion = 0.0f;

			// 강체가 세계공간에서 영향을 받지않도록 조절합니다.
			bool mIsAwake = false;
			// 강체가 잠들지 못하도록 합니다.
			bool mCanSleep = false;

			// 세계공간에서의 강체의 위치
			DirectX::XMFLOAT4 mPosition = {};
			// 세계공간에서의 강체의 방향
			DirectX::XMFLOAT4 mOrientation = {};
			// 세계공간에서의 강체의 선형 속도
			DirectX::XMFLOAT4 mVelocity = {};
			// 세계공간에서의 강체의 회전 속도, 또는 회전
			DirectX::XMFLOAT4 mRotation = {};
			// 다음 적분단계에서 사용할 힘을 저장
			DirectX::XMFLOAT4 mForceAccum = {};
			// 다음 적분단계에서 사용할 토크를 저장
			DirectX::XMFLOAT4 mTorqueAccum = {};
			// 강체의 선가속도
			DirectX::XMFLOAT4 mAcceleration = {};
			// 전 프레임에서의 선가속도를 저장
			DirectX::XMFLOAT4 mLastframeAcceleration = {};

			// 신체공간에서의 관성텐서의 역행렬
			DirectX::XMFLOAT4X4 mInverseInertiaTensor = {};
			// 세계공간에서의 관성텐서의 역행렬
			DirectX::XMFLOAT4X4 mInverseInertiaTensorWorld = {};
			// 신체공간을 세계공간으로 바꾸는 변환 행렬
			DirectX::XMFLOAT4X4 mTransformMatrix = {};

		};
	}
}

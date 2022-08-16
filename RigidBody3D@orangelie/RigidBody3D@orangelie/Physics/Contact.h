#pragma once

#include "RigidBody.h"


namespace orangelie
{
	namespace Physics
	{
		class ContactResolver;

		class Contact
		{
			friend class ContactResolver;

		public:
			// 접촉에 관련된 강체들
			RigidBody* mBodies[2];

			// 마찰 계수
			float mFriction;
			// 반발 계수
			float mRestitution;
			// 침투 깊이
			float mPenetration;

			// 세계좌표에서의 접촉점
			DirectX::XMFLOAT4 mContactPoint;
			// 세계좌표에서의 방향
			DirectX::XMFLOAT4 mContactNormal;

			void SetBodyData(RigidBody* one, RigidBody* two, float friction, float restitution);

		protected:
			// 접촉을 해결하는데 필요한 속도변화
			float mDesiredDeltaVelocity;

			// 접촉-세계 변환행렬
			DirectX::XMFLOAT4X4 mContactToWorld;

			// 접촉했을때의 닫힌 속력
			DirectX::XMFLOAT4 mContactVelocity;
			// 강체중심을 기준으로한 접점의 세계공간 위치
			DirectX::XMFLOAT4 mRelativeContactPosition[2];

		protected:
			// 내부데이터에서 상태데이터로 전환
			void CalculateInternals(float dt);
			// 두 접촉을 반전하여, 강체를 교환
			void SwapBodies();
			/**
				[Awake = true] 가 되어있는 강체의 두 접점에서 발생하는 상태를 업데이트.
				[Awake = true]인 강체와 접촉한다면, 피접촉체도 [Awake = true]가 된다.
			*/
			void MatchAwakeState();
			// 접촉해결에 필요한 속도변화를 계산하고 설정
			void CalculateDesiredDeltaVelocity(float dt);
			// 지정된 강체에 있는 접점의 속도를 계산하고 반환
			DirectX::XMFLOAT4 CalculateLocalVelocity(unsigned bodyIndex, float dt);
			/**
				1차 마찰 방향(등방성 마찰의 경우) 또는
				랜덤 방향(등방성 마찰의 경우)을 기준으로 접점에 대한 직교 기준을 계산
			*/
			void CalculateContactBasis();
			// 주어진 강체에 대하여 힘을 가하여, 속력을 변화시킨다.
			void ApplyImpulse(const DirectX::XMFLOAT4& impulse, RigidBody* body,
				DirectX::XMFLOAT4* velocityChange, DirectX::XMFLOAT4* rotationChange);
			// 이 접점의 관성-가중 Impulse 기반 분해능만 수행합니다.
			void ApplyVelocityChange(DirectX::XMFLOAT4 velocityChange[2], DirectX::XMFLOAT4 rotationChange[2]);
			// 이 접점의 관성-가중 침투 분해능만 수행합니다.
			void ApplyPositionChange(DirectX::XMFLOAT4 linearChange[2], DirectX::XMFLOAT4 angularChange[2], float penetration);
			// 접촉에 마찰이 없는 경우 이때 가해지는 힘을 구한다.
			DirectX::XMFLOAT4 CalculateFrictionlessImpulse(DirectX::XMFLOAT4X4* inverseInertiaTensor);
			// 접촉에 마찰이 0이 아닌 경우 이때 가해지는 힘을 구한다.
			DirectX::XMFLOAT4 CalculateFrictionImpulse(DirectX::XMFLOAT4X4* inverseInertiaTensor);

		};

		class ContactResolver
		{
		public:


		private:


		};
	}
}
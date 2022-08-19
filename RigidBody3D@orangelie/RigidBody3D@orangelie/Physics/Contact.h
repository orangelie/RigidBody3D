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
			// ���˿� ���õ� ��ü��
			RigidBody* mBodies[2];

			// ���� ���
			float mFriction;
			// �ݹ� ���
			float mRestitution;
			// ħ�� ����
			float mPenetration;

			// ������ǥ������ ������
			DirectX::XMFLOAT4 mContactPoint;
			// ������ǥ������ ����
			DirectX::XMFLOAT4 mContactNormal;

			void SetBodyData(RigidBody* one, RigidBody* two, float friction, float restitution);

		protected:
			// ������ �ذ��ϴµ� �ʿ��� �ӵ���ȭ
			float mDesiredDeltaVelocity;

			// ����-���� ��ȯ���
			DirectX::XMFLOAT4X4 mContactToWorld;

			// ������������ ���� �ӷ�
			DirectX::XMFLOAT4 mContactVelocity;
			// ��ü�߽��� ���������� ������ ������� ��ġ
			DirectX::XMFLOAT4 mRelativeContactPosition[2];

		protected:
			// ���ε����Ϳ��� ���µ����ͷ� ��ȯ
			void CalculateInternals(float dt);
			// �� ������ �����Ͽ�, ��ü�� ��ȯ
			void SwapBodies();
			/**
				[Awake = true] �� �Ǿ��ִ� ��ü�� �� �������� �߻��ϴ� ���¸� ������Ʈ.
				[Awake = true]�� ��ü�� �����Ѵٸ�, ������ü�� [Awake = true]�� �ȴ�.
			*/
			void MatchAwakeState();
			// �����ذῡ �ʿ��� �ӵ���ȭ�� ����ϰ� ����
			void CalculateDesiredDeltaVelocity(float dt);
			// ������ ��ü�� �ִ� ������ �ӵ��� ����ϰ� ��ȯ
			DirectX::XMFLOAT4 CalculateLocalVelocity(unsigned bodyIndex, float dt);
			/**
				1�� ���� ����(��漺 ������ ���) �Ǵ�
				���� ����(��漺 ������ ���)�� �������� ������ ���� ���� ������ ���
			*/
			void CalculateContactBasis();
			// �־��� ��ü�� ���Ͽ� ���� ���Ͽ�, �ӷ��� ��ȭ��Ų��.
			void ApplyImpulse(const DirectX::XMFLOAT4& impulse, RigidBody* body,
				DirectX::XMFLOAT4* velocityChange, DirectX::XMFLOAT4* rotationChange);
			// �� ������ ����-���� Impulse ��� ���شɸ� �����մϴ�.
			void ApplyVelocityChange(DirectX::XMFLOAT4 velocityChange[2], DirectX::XMFLOAT4 rotationChange[2]);
			// �� ������ ����-���� ħ�� ���شɸ� �����մϴ�.
			void ApplyPositionChange(DirectX::XMFLOAT4 linearChange[2], DirectX::XMFLOAT4 angularChange[2], float penetration);
			// ���˿� ������ ���� ��� �̶� �������� ���� ���Ѵ�.
			DirectX::XMFLOAT4 CalculateFrictionlessImpulse(DirectX::XMFLOAT4X4* inverseInertiaTensor);
			// ���˿� ������ 0�� �ƴ� ��� �̶� �������� ���� ���Ѵ�.
			DirectX::XMFLOAT4 CalculateFrictionImpulse(DirectX::XMFLOAT4X4* inverseInertiaTensor);

		};

		class ContactResolver
		{
		protected:
			unsigned mVelocityIterations;
			unsigned mPositionIterations;

			float mVelocityEpsilon;
			float mPositionEpsilon;

		public:
			unsigned mVelocityIterationsUsed;
			unsigned mPositionIterationsUsed;

		private:
			unsigned mValidSettings;

		public:
			ContactResolver(unsigned iterations, float velocityEpsilon = 0.01f, float positionEpsilon = 0.01f);
			ContactResolver(unsigned velocityIterations, unsigned positionIterations, float velocityEpsilon = 0.01f, float positionEpsilon = 0.01f);

			bool isValid()
			{
				return ((mVelocityIterations > 0) &&
					(mPositionIterations > 0) &&
					(mVelocityEpsilon >= 0.0f) &&
					(mPositionEpsilon >= 0.0f));
			}

			void SetIterations(unsigned iterations);
			void SetIterations(unsigned velocityIterations, unsigned positionIterations);
			void SetEpsilon(float velocityEpsilon, float positionEpsilon);

			void ResolveContacts(Contact* contactArray, unsigned numContacts, float dt);

		protected:
			void PrepareContacts(Contact* contactArray, unsigned numContacts, float dt);
			void AdjustPositions(Contact* contactArray, unsigned numContacts, float dt);
			void AdjustVelocities(Contact* contactArray, unsigned numContacts, float dt);

		};
	}
}
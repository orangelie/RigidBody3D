#include "Contact.h"


namespace orangelie
{
	namespace Physics
	{
		void Contact::SetBodyData(RigidBody* one, RigidBody* two, float friction, float restitution)
		{
			mBodies[0] = one;
			mBodies[1] = two;
			mFriction = friction;
			mRestitution = restitution;
		}

		void Contact::CalculateInternals(float dt)
		{
			DirectX::XMFLOAT4 body1Pos = {}, body2Pos = {};
			mBodies[0]->GetPosition(body1Pos); mBodies[1]->GetPosition(body2Pos);

			if (mBodies[0])
			{
				SwapBodies();
			}
			assert(mBodies[0]);

			CalculateContactBasis();

			DirectX::XMStoreFloat4(&mRelativeContactPosition[0], DirectX::XMVectorSubtract(DirectX::XMLoadFloat4(&mContactPoint), DirectX::XMLoadFloat4(&body1Pos)));
			if (mBodies[1])
			{
				DirectX::XMStoreFloat4(&mRelativeContactPosition[1], DirectX::XMVectorSubtract(DirectX::XMLoadFloat4(&mContactPoint), DirectX::XMLoadFloat4(&body2Pos)));
			}

			mContactVelocity = CalculateLocalVelocity(0, dt);
			if (mBodies[1])
			{
				DirectX::XMStoreFloat4(&mContactVelocity, DirectX::XMVectorSubtract(DirectX::XMLoadFloat4(&mContactVelocity), DirectX::XMLoadFloat4(&CalculateLocalVelocity(1, dt))));
			}

			CalculateDesiredDeltaVelocity(dt);
		}

		void Contact::SwapBodies()
		{
			mContactNormal.x *= -1.0f; mContactNormal.y *= -1.0f; mContactNormal.z *= -1.0f;

			RigidBody* tempBody = mBodies[0];
			mBodies[0] = mBodies[1];
			mBodies[1] = tempBody;
		}

		void Contact::MatchAwakeState()
		{
			if (!mBodies[1])
			{
				return;
			}

			bool body0Awake = mBodies[0]->GetAwake();
			bool body1Awake = mBodies[1]->GetAwake();

			if (body0Awake ^ body1Awake)
			{
				if (body0Awake)
				{
					mBodies[1]->setAwake();
				}
				else
				{
					mBodies[0]->setAwake();
				}
			}
		}

		void Contact::CalculateDesiredDeltaVelocity(float dt)
		{
			const float velocityLimit = 0.25f;
			float velocityFromAcc = 0.0f;
			DirectX::XMFLOAT4 lastFrameAcc = {};

			if (mBodies[0]->GetAwake())
			{
				mBodies[0]->GetLastframeAcceleration(lastFrameAcc);
				lastFrameAcc.x *= dt; lastFrameAcc.y *= dt; lastFrameAcc.z *= dt;

				velocityFromAcc += DirectX::XMVectorGetX(
					DirectX::XMVector3Dot(DirectX::XMLoadFloat4(&lastFrameAcc), DirectX::XMLoadFloat4(&mContactNormal)));
			}

			if (mBodies[1] && mBodies[1]->GetAwake())
			{
				mBodies[1]->GetLastframeAcceleration(lastFrameAcc);
				lastFrameAcc.x *= dt; lastFrameAcc.y *= dt; lastFrameAcc.z *= dt;

				velocityFromAcc -= DirectX::XMVectorGetX(
					DirectX::XMVector3Dot(DirectX::XMLoadFloat4(&lastFrameAcc), DirectX::XMLoadFloat4(&mContactNormal)));
			}

			float thisRestitution = mRestitution;
			if (std::fabsf(mContactVelocity.x) < velocityLimit)
			{
				thisRestitution = 0.0f;
			}

			mDesiredDeltaVelocity = -mContactVelocity.x - thisRestitution * (mContactVelocity.x - velocityFromAcc);
		}

		DirectX::XMFLOAT4 Contact::CalculateLocalVelocity(unsigned bodyIndex, float dt)
		{
			using namespace DirectX;
			using namespace DirectX::PackedVector;

			RigidBody* body = mBodies[bodyIndex];

			DirectX::XMFLOAT4 rot = {}, vel = {}, velocity = {}, contactVelocity = {}, accVelocity = {};
			body->GetRotation(rot); body->GetVelocity(vel);

			DirectX::XMStoreFloat4(&velocity,
				DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&rot), DirectX::XMLoadFloat4(&mRelativeContactPosition[bodyIndex])));
			velocity.x += vel.x; velocity.y += vel.y; velocity.z += vel.z;

			Utils::MathTool::TransformTranspose(contactVelocity, mContactToWorld, velocity);
			body->GetLastframeAcceleration(accVelocity); accVelocity.x *= dt; accVelocity.y *= dt; accVelocity.z *= dt;

			Utils::MathTool::TransformTranspose(accVelocity, mContactToWorld, accVelocity);
			accVelocity.x = 0.0f;

			DirectX::XMStoreFloat4(&contactVelocity,
				DirectX::XMVectorAdd(DirectX::XMLoadFloat4(&contactVelocity), DirectX::XMLoadFloat4(&accVelocity)));

			return contactVelocity;
		}

		void Contact::CalculateContactBasis()
		{
			DirectX::XMFLOAT4 contactTangent[2] = {};

			if (std::fabsf(mContactNormal.x) > std::fabsf(mContactNormal.x))
			{
				const float s = 1.0f / std::sqrt(mContactNormal.z * mContactNormal.z + mContactNormal.x * mContactNormal.x);

				contactTangent[0].x = s * mContactNormal.z;
				contactTangent[0].y = 0.0f;
				contactTangent[0].z = -s * mContactNormal.x;

				contactTangent[1].x = mContactNormal.y * contactTangent[0].x;
				contactTangent[1].y = mContactNormal.z * contactTangent[0].x - mContactNormal.x * contactTangent[0].z;
				contactTangent[1].z = -mContactNormal.y * contactTangent[0].x;
			}
			else
			{
				const float s = 1.0f / std::sqrt(mContactNormal.z * mContactNormal.z + mContactNormal.y * mContactNormal.y);

				contactTangent[0].x = 0.0f;
				contactTangent[0].y = -s * mContactNormal.z;
				contactTangent[0].z = s * mContactNormal.y;

				contactTangent[1].x = mContactNormal.y * contactTangent[0].z - mContactNormal.z * contactTangent[0].y;
				contactTangent[1].y = -mContactNormal.x * contactTangent[0].z;
				contactTangent[1].z = mContactNormal.x * contactTangent[0].y;
			}

			Utils::MathTool::SetComponents(mContactToWorld, mContactNormal, contactTangent[0], contactTangent[1]);
		}

		void Contact::ApplyVelocityChange(DirectX::XMFLOAT4 velocityChange[2], DirectX::XMFLOAT4 rotationChange[2])
		{

		}

		void Contact::ApplyPositionChange(DirectX::XMFLOAT4 linearChange[2], DirectX::XMFLOAT4 angularChange[2], float penetration)
		{

		}

		DirectX::XMFLOAT4 Contact::CalculateFrictionlessImpulse(DirectX::XMFLOAT4X4* inverseInertiaTensor)
		{
			DirectX::XMFLOAT4 impulseContact = {}, deltaVelWorld = {};

			DirectX::XMStoreFloat4(&deltaVelWorld, DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&mRelativeContactPosition[0]),
				DirectX::XMLoadFloat4(&mContactNormal)));
			Utils::MathTool::Transform(deltaVelWorld, inverseInertiaTensor[0], deltaVelWorld);
			DirectX::XMStoreFloat4(&deltaVelWorld, DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&deltaVelWorld),
				DirectX::XMLoadFloat4(&mRelativeContactPosition[0])));

			float deltaVelocity = DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMLoadFloat4(&deltaVelWorld),
				DirectX::XMLoadFloat4(&mContactNormal)));
			deltaVelocity += mBodies[0]->GetInverseMass();

			
			if (mBodies[1])
			{
				DirectX::XMFLOAT4 deltaVelWorld = {};

				DirectX::XMStoreFloat4(&deltaVelWorld, DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&mRelativeContactPosition[1]),
					DirectX::XMLoadFloat4(&mContactNormal)));
				Utils::MathTool::Transform(deltaVelWorld, inverseInertiaTensor[1], deltaVelWorld);
				DirectX::XMStoreFloat4(&deltaVelWorld, DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&deltaVelWorld),
					DirectX::XMLoadFloat4(&mRelativeContactPosition[1])));

				deltaVelocity += DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMLoadFloat4(&deltaVelWorld),
					DirectX::XMLoadFloat4(&mContactNormal)));
				deltaVelocity += mBodies[1]->GetInverseMass();
			}

			impulseContact.x = mDesiredDeltaVelocity / deltaVelocity;
			impulseContact.y = 0.0f;
			impulseContact.z = 0.0f;

			return impulseContact;
		}

		DirectX::XMFLOAT4 Contact::CalculateFrictionImpulse(DirectX::XMFLOAT4X4* inverseInertiaTensor)
		{
			DirectX::XMFLOAT4 impulseContact = {};
			float inverseMass = mBodies[0]->GetInverseMass();




			return impulseContact;
		}
	}
}
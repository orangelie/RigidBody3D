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
				const float sqrt = std::sqrtf(mContactNormal.z * mContactNormal.z + mContactNormal.x * mContactNormal.x);
				float s = 1.0 / sqrt;
				if (sqrt == 0.0f)
					s = 0.0f;

				contactTangent[0].x = s * mContactNormal.z;
				contactTangent[0].y = 0.0f;
				contactTangent[0].z = -s * mContactNormal.x;

				contactTangent[1].x = mContactNormal.y * contactTangent[0].x;
				contactTangent[1].y = mContactNormal.z * contactTangent[0].x - mContactNormal.x * contactTangent[0].z;
				contactTangent[1].z = -mContactNormal.y * contactTangent[0].x;
			}
			else
			{
				const float sqrt = std::sqrtf(mContactNormal.z * mContactNormal.z + mContactNormal.y * mContactNormal.y);
				float s = 1.0 / sqrt;
				if (sqrt == 0.0f)
					s = 0.0f;

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
			DirectX::XMFLOAT3X4 inverseInertiaTensorWorld[2] = {};
			DirectX::XMFLOAT4 impulseContact = {};

			mBodies[0]->GetInverseInertiaTensorWorld(inverseInertiaTensorWorld[0]);
			if(mBodies[1])
				mBodies[1]->GetInverseInertiaTensorWorld(inverseInertiaTensorWorld[1]);

			if (mFriction == 0.0f)
			{
				impulseContact = CalculateFrictionlessImpulse(inverseInertiaTensorWorld);
			}
			else
			{
				impulseContact = CalculateFrictionImpulse(inverseInertiaTensorWorld);
			}

			DirectX::XMFLOAT4 impulse = {};
			Utils::MathTool::Transform(impulse, mContactToWorld, impulseContact);

			DirectX::XMFLOAT4 impulsiveTorque = {};
			DirectX::XMStoreFloat4(&impulsiveTorque, DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&mRelativeContactPosition[0]), DirectX::XMLoadFloat4(&impulse)));
			Utils::MathTool::Transform(rotationChange[0], inverseInertiaTensorWorld[0], impulsiveTorque);
			velocityChange[0].x = velocityChange[0].y = velocityChange[0].z = 0.0f;
			Utils::MathTool::AddScaledVector(velocityChange[0], impulse, mBodies[0]->GetInverseMass());

			mBodies[0]->AddVelocity(velocityChange[0].x, velocityChange[0].y, velocityChange[0].z);
			mBodies[0]->AddRotation(rotationChange[0].x, rotationChange[0].y, rotationChange[0].z);

			if (mBodies[1])
			{
				DirectX::XMFLOAT4 impulsiveTorque = {};
				DirectX::XMStoreFloat4(&impulsiveTorque, DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&impulse), DirectX::XMLoadFloat4(&mRelativeContactPosition[1])));
				Utils::MathTool::Transform(rotationChange[1], inverseInertiaTensorWorld[1], impulsiveTorque);
				velocityChange[1].x = velocityChange[1].y = velocityChange[1].z = 0.0f;
				Utils::MathTool::AddScaledVector(velocityChange[1], impulse, -mBodies[1]->GetInverseMass());

				mBodies[1]->AddVelocity(velocityChange[1].x, velocityChange[1].y, velocityChange[1].z);
				mBodies[1]->AddRotation(rotationChange[1].x, rotationChange[1].y, rotationChange[1].z);
			}
		}

		void Contact::ApplyPositionChange(DirectX::XMFLOAT4 linearChange[2], DirectX::XMFLOAT4 angularChange[2], float penetration)
		{
			const float angularLimit = 0.2f;
			float totalInertia = 0.0f;
			float linearMove[2] = {}, angularMove[2] = {};
			float linearInertia[2] = {}, angularInertia[2] = {};

			for (unsigned i = 0; i < 2; ++i) if (mBodies[i])
			{
				DirectX::XMFLOAT3X4 inverseInertiaTensorWorld;
				mBodies[i]->GetInverseInertiaTensorWorld(inverseInertiaTensorWorld);

				DirectX::XMFLOAT4 angularInertiaWorld;
				DirectX::XMStoreFloat4(&angularInertiaWorld,
					DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&mRelativeContactPosition[i]), DirectX::XMLoadFloat4(&mContactNormal)));
				Utils::MathTool::Transform(angularInertiaWorld, inverseInertiaTensorWorld, angularInertiaWorld);
				DirectX::XMStoreFloat4(&angularInertiaWorld,
					DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&angularInertiaWorld), DirectX::XMLoadFloat4(&mRelativeContactPosition[i])));

				angularInertia[i] = DirectX::XMVectorGetX(DirectX::XMVector3Dot(DirectX::XMLoadFloat4(&angularInertiaWorld), DirectX::XMLoadFloat4(&mContactNormal)));
				linearInertia[i] = mBodies[i]->GetInverseMass();

				totalInertia += linearInertia[i] + angularInertia[i];
			}

			for (unsigned i = 0; i < 2; ++i) if (mBodies[i])
			{
				float sign = (i == 0) ? 1.0f : -1.0f;

				linearMove[i] = mPenetration * sign * (linearInertia[i] / totalInertia);
				angularMove[i] = mPenetration * sign * (angularInertia[i] / totalInertia);

				DirectX::XMFLOAT4 projection = mRelativeContactPosition[i];

				Utils::MathTool::AddScaledVector(projection, mContactNormal, -DirectX::XMVectorGetX(
					DirectX::XMVector3Dot(DirectX::XMLoadFloat4(&mRelativeContactPosition[i]), DirectX::XMLoadFloat4(&mContactNormal))));


				float maxMagnitude = angularLimit * DirectX::XMVectorGetX(DirectX::XMVector3Length(DirectX::XMLoadFloat4(&projection)));

				if (angularMove[i] < -maxMagnitude)
				{
					float totalMove = linearMove[i] + angularMove[i];
					angularMove[i] = -maxMagnitude;
					linearMove[i] = totalMove - angularMove[i];

				}
				else if (angularMove[i] > maxMagnitude)
				{
					float totalMove = linearMove[i] + angularMove[i];
					angularMove[i] = maxMagnitude;
					linearMove[i] = totalMove - angularMove[i];
				}

				if (angularMove[i] == 0)
				{
					angularChange[i].x = angularChange[i].y = angularChange[i].z = 0.0f;
				}
				else
				{
					DirectX::XMFLOAT3X4 inverseInertiaTensorWorld;
					mBodies[i]->GetInverseInertiaTensorWorld(inverseInertiaTensorWorld);

					DirectX::XMFLOAT4 targetAngularDirection;
					DirectX::XMStoreFloat4(&targetAngularDirection,
						DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&mRelativeContactPosition[i]), DirectX::XMLoadFloat4(&mContactNormal)));

					
					Utils::MathTool::Transform(angularChange[i], inverseInertiaTensorWorld, targetAngularDirection);
					Utils::MathTool::Scalar(angularChange[i], angularMove[i] / angularInertia[i]);
				}


				linearChange[i].x = mContactNormal.x * linearMove[i];
				linearChange[i].y = mContactNormal.y * linearMove[i];
				linearChange[i].z = mContactNormal.z * linearMove[i];

				DirectX::XMFLOAT4 position = {};
				mBodies[i]->GetPosition(position);
				Utils::MathTool::AddScaledVector(position, mContactNormal, linearMove[i]);
				mBodies[i]->SetPosition(position.x, position.y, position.z);

				DirectX::XMFLOAT4 orientation = {};
				mBodies[i]->GetOrientation(orientation);
				Utils::MathTool::AddScaledQuaternion(orientation, angularChange[i], 1.0f);
				mBodies[i]->SetOrientation(orientation.x, orientation.y, orientation.z, orientation.w);

				if (mBodies[i]->GetAwake() == false) {
					mBodies[i]->CalculateDerivedData();
				}
			}
		}

		DirectX::XMFLOAT4 Contact::CalculateFrictionlessImpulse(DirectX::XMFLOAT3X4* inverseInertiaTensor)
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

		DirectX::XMFLOAT4 Contact::CalculateFrictionImpulse(DirectX::XMFLOAT3X4* inverseInertiaTensor)
		{
			DirectX::XMFLOAT4 impulseContact = {};
			float inverseMass = mBodies[0]->GetInverseMass();

			DirectX::XMFLOAT3X4 impulseToTorque = Utils::MathTool::SetSkewSymmetric(mRelativeContactPosition[0]);

			DirectX::XMMATRIX deltaVelWorld1 = DirectX::XMLoadFloat3x4(&impulseToTorque);
			deltaVelWorld1 *= DirectX::XMLoadFloat3x4(&inverseInertiaTensor[0]);
			deltaVelWorld1 *= DirectX::XMLoadFloat3x4(&impulseToTorque);
			deltaVelWorld1 *= -1.0f;

			if (mBodies[1])
			{
				impulseToTorque = Utils::MathTool::SetSkewSymmetric(mRelativeContactPosition[1]);

				DirectX::XMMATRIX deltaVelWorld2 = DirectX::XMLoadFloat3x4(&impulseToTorque);
				deltaVelWorld2 *= DirectX::XMLoadFloat3x4(&inverseInertiaTensor[0]);
				deltaVelWorld2 *= DirectX::XMLoadFloat3x4(&impulseToTorque);
				deltaVelWorld2 *= -1.0f;

				deltaVelWorld1 += deltaVelWorld2;
				inverseMass += mBodies[1]->GetInverseMass();
			}

			DirectX::XMMATRIX contactToWorld = DirectX::XMLoadFloat3x4(&mContactToWorld);

			DirectX::XMMATRIX deltaVelocity = DirectX::XMMatrixTranspose(contactToWorld);
			deltaVelocity *= deltaVelWorld1;
			deltaVelocity *= contactToWorld;

			DirectX::XMFLOAT3X4 deltaVelocityF = {};

			DirectX::XMStoreFloat3x4(&deltaVelocityF, deltaVelocity);
			deltaVelocityF.m[0][0] += inverseMass;
			deltaVelocityF.m[1][1] += inverseMass;
			deltaVelocityF.m[2][2] += inverseMass;

			DirectX::XMFLOAT3X4 impulseMatrix = {};
			DirectX::XMMATRIX tempMat = DirectX::XMLoadFloat3x4(&deltaVelocityF);
			DirectX::XMStoreFloat3x4(&impulseMatrix, DirectX::XMMatrixInverse(&DirectX::XMMatrixDeterminant(tempMat), tempMat));
			//impulseMatrix = Utils::MathTool::SetInverse(deltaVelocityF);

			DirectX::XMFLOAT4 velKill = { mDesiredDeltaVelocity, -mContactVelocity.y,  -mContactVelocity.z, 0.0f };
			Utils::MathTool::Transform(impulseContact, impulseMatrix, velKill);

			float planarImpulse = std::sqrtf(impulseContact.y * impulseContact.y + impulseContact.z * impulseContact.z);
			if (planarImpulse > impulseContact.x * mFriction)
			{
				impulseContact.y /= planarImpulse;
				impulseContact.z /= planarImpulse;

				impulseContact.x = deltaVelocityF.m[0][0] + deltaVelocityF.m[0][1] * mFriction * impulseContact.y + deltaVelocityF.m[0][2] * mFriction * impulseContact.z;
				impulseContact.x = mDesiredDeltaVelocity / impulseContact.x;
				impulseContact.y *= impulseContact.x * mFriction;
				impulseContact.z *= impulseContact.x * mFriction;
			}
			
			return impulseContact;
		}


		ContactResolver::ContactResolver(unsigned iterations, float velocityEpsilon, float positionEpsilon)
		{
			SetIterations(iterations);
			SetEpsilon(velocityEpsilon, positionEpsilon);
		}

		ContactResolver::ContactResolver(unsigned velocityIterations, unsigned positionIterations, float velocityEpsilon, float positionEpsilon)
		{
			SetIterations(velocityIterations, positionIterations);
			SetEpsilon(velocityEpsilon, positionEpsilon);
		}

		void ContactResolver::SetIterations(unsigned iterations)
		{
			SetIterations(iterations, iterations);
		}

		void ContactResolver::SetIterations(unsigned velocityIterations, unsigned positionIterations)
		{
			mVelocityIterations = velocityIterations;
			mPositionIterations = positionIterations;
		}

		void ContactResolver::SetEpsilon(float velocityEpsilon, float positionEpsilon)
		{
			mVelocityEpsilon = velocityEpsilon;
			mPositionEpsilon = positionEpsilon;
		}

		void ContactResolver::ResolveContacts(Contact* contactArray, unsigned numContacts, float dt)
		{
			if (numContacts <= 0)
				return;
			if (!isValid())
				return;

			PrepareContacts(contactArray, numContacts, dt);
			AdjustPositions(contactArray, numContacts, dt);
			AdjustVelocities(contactArray, numContacts, dt);
		}

		void ContactResolver::PrepareContacts(Contact* contactArray, unsigned numContacts, float dt)
		{
			Contact* lastContact = contactArray + numContacts;

			for (Contact* contact = contactArray; contact < lastContact; ++contact)
			{
				contact->CalculateInternals(dt);
			}
		}

		void ContactResolver::AdjustPositions(Contact* contactArray, unsigned numContacts, float dt)
		{
			using namespace DirectX;

			DirectX::XMFLOAT4 linearChange[2] = {}, angularChange[2] = {};
			DirectX::XMVECTOR deltaVel = {};

			mPositionIterationsUsed = 0;
			while (mPositionIterationsUsed < mPositionIterations)
			{
				float max = mPositionEpsilon;
				unsigned index = numContacts;

				for (unsigned i = 0; i < numContacts; ++i)
				{
					if (contactArray[i].mPenetration > max)
					{
						max = contactArray[i].mPenetration;
						index = i;
					}
				}

				if (index == numContacts)
					break;

				contactArray[index].MatchAwakeState();
				contactArray[index].ApplyPositionChange(linearChange, angularChange, max);


				for (unsigned i = 0; i < numContacts; ++i)
				{
					for (unsigned b = 0; b < 2; ++b) if(contactArray[i].mBodies[b])
					{
						for (unsigned d = 0; d < 2; ++d)
						{
							if (contactArray[i].mBodies[b] == contactArray[index].mBodies[d])
							{
								deltaVel = DirectX::XMLoadFloat4(&linearChange[d]) + DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&angularChange[d]),
									DirectX::XMLoadFloat4(&contactArray[i].mRelativeContactPosition[b]));

								contactArray[i].mPenetration += DirectX::XMVectorGetX(DirectX::XMVector3Dot(deltaVel,
									DirectX::XMLoadFloat4(&contactArray[i].mContactNormal))) * (b ? 1.0f : -1.0f);
							}
						}
					}
				}

				++mPositionIterationsUsed;
			}
		}

		void ContactResolver::AdjustVelocities(Contact* contactArray, unsigned numContacts, float dt)
		{
			using namespace DirectX;

			DirectX::XMFLOAT4 velocityChange[2] = {}, rotationChange[2] = {};
			DirectX::XMFLOAT4 deltaVel = {}, tmpVel = {};

			mVelocityIterationsUsed = 0;
			while (mVelocityIterationsUsed < mVelocityIterations)
			{
				float max = mVelocityEpsilon;
				unsigned index = numContacts;

				for (unsigned i = 0; i < numContacts; ++i)
				{
					if (contactArray[i].mDesiredDeltaVelocity > max)
					{
						max = contactArray[i].mDesiredDeltaVelocity;
						index = i;
					}
				}

				if (index == numContacts)
					break;

				contactArray[index].MatchAwakeState();
				contactArray[index].ApplyVelocityChange(velocityChange, rotationChange);


				for (unsigned i = 0; i < numContacts; ++i)
				{
					for (unsigned b = 0; b < 2; ++b) if (contactArray[i].mBodies[b])
					{
						for (unsigned d = 0; d < 2; ++d)
						{
							if (contactArray[i].mBodies[b] == contactArray[index].mBodies[d])
							{
								DirectX::XMStoreFloat4(&deltaVel, DirectX::XMLoadFloat4(&velocityChange[d]) + DirectX::XMVector3Cross(DirectX::XMLoadFloat4(&rotationChange[d]),
									DirectX::XMLoadFloat4(&contactArray[i].mRelativeContactPosition[b])));

								Utils::MathTool::TransformTranspose(tmpVel, contactArray[i].mContactToWorld, deltaVel);
								contactArray[i].mContactVelocity.x += tmpVel.x * (b ? -1.0f : 1.0f);
								contactArray[i].mContactVelocity.y += tmpVel.y * (b ? -1.0f : 1.0f);
								contactArray[i].mContactVelocity.z += tmpVel.z * (b ? -1.0f : 1.0f);

								contactArray[i].CalculateDesiredDeltaVelocity(dt);
							}
						}
					}
				}

				++mVelocityIterationsUsed;
			}
		}
	}
}
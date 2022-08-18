#include "RigidBody.h"


namespace orangelie
{
	namespace Physics
	{
		void RigidBody::CalculateDerivedData()
		{
			DirectX::XMVECTOR position = DirectX::XMLoadFloat4(&mPosition);
			DirectX::XMVECTOR quaternion = DirectX::XMLoadFloat4(&mOrientation);
			DirectX::XMVECTOR quatOrigin = DirectX::XMQuaternionIdentity();

			quaternion = DirectX::XMQuaternionNormalize(quaternion);

			DirectX::XMMATRIX transformMat = DirectX::XMMatrixAffineTransformation(DirectX::XMVectorSet(1.0f, 1.0f, 1.0f, 0.0f), quatOrigin, quaternion, position);
			DirectX::XMStoreFloat4x4(&mTransformMatrix, DirectX::XMMatrixTranspose(transformMat));

			mTransformMatrix.m[0][3] = mPosition.x;
			mTransformMatrix.m[1][3] = mPosition.y;
			mTransformMatrix.m[2][3] = mPosition.z;


			DirectX::XMMATRIX A = DirectX::XMLoadFloat4x4(&mTransformMatrix);
			DirectX::XMMATRIX B = DirectX::XMLoadFloat4x4(&mInverseInertiaTensor);
			B = DirectX::XMMatrixMultiply(A, B);
			DirectX::XMStoreFloat4x4(&mInverseInertiaTensorWorld, DirectX::XMMatrixMultiply(B, DirectX::XMMatrixTranspose(A)));

			DirectX::XMStoreFloat4(&mPosition, position);
			DirectX::XMStoreFloat4(&mOrientation, quaternion);
		}

		void RigidBody::Integrate(float dt)
		{
			if (mIsAwake == false)
			{
				return;
			}

			// Acceleration Update
			mLastframeAcceleration = mAcceleration;
			Utils::MathTool::AddScaledVector(mLastframeAcceleration, mForceAccum, mInverseMass);

			DirectX::XMFLOAT4 angularAcceleration = {};
			Utils::MathTool::Transform(angularAcceleration, mInverseInertiaTensorWorld, mTorqueAccum);
			
			// Velocity Update
			Utils::MathTool::AddScaledVector(mVelocity, mLastframeAcceleration, dt);
			Utils::MathTool::AddScaledVector(mRotation, angularAcceleration, dt);

			// Velocity Drag Damping
			Utils::MathTool::Scalar(mVelocity, std::powf(mLinearDamping, dt));
			Utils::MathTool::Scalar(mRotation, std::powf(mAngularDamping, dt));
			
			// Position Update
			Utils::MathTool::AddScaledVector(mPosition, mVelocity, dt);
			Utils::MathTool::AddScaledQuaternion(mOrientation, mRotation, dt);

			CalculateDerivedData();
			ClearAccmulator();

			if (mCanSleep)
			{
				float motion =
					(mVelocity.x * mVelocity.x + mVelocity.y * mVelocity.y + mVelocity.z * mVelocity.z) +
					(mRotation.x * mRotation.x + mRotation.y * mRotation.y + mRotation.z * mRotation.z);
				float bias = std::powf(0.5f, dt);

				mMotion = mMotion * bias + motion * (1.0f - bias);

				if (mMotion < gSleepEpsilon)
					setAwake(false);
				else if (mMotion > 10.0f * gSleepEpsilon)
					mMotion = 10.0f * gSleepEpsilon;
			}
		}

		void RigidBody::ClearAccmulator()
		{
			mForceAccum.x = mForceAccum.y = mForceAccum.z = 0.0f;
			mTorqueAccum.x = mTorqueAccum.y = mTorqueAccum.z = 0.0f;
		}

		void RigidBody::AddForce(float x, float y, float z)
		{
			mForceAccum.x += x;
			mForceAccum.y += y;
			mForceAccum.z += z;
		}

		void RigidBody::SetPosition(const float x, const float y, const float z)
		{
			mPosition.x = x;
			mPosition.y = y;
			mPosition.z = z;
			mPosition.w = 0.0f;
		}

		void RigidBody::setAwake(const bool awake)
		{
			if (awake)
			{
				mIsAwake = true;
				mMotion = gSleepEpsilon * 2.0f;
			}
			else
			{
				mIsAwake = false;
				mVelocity.x = mVelocity.y = mVelocity.z = 0.0f;
				mRotation.x = mRotation.y = mRotation.z = 0.0f;
			}
		}

		void RigidBody::setSleep(const bool sleep)
		{
			mCanSleep = sleep;

			if (mCanSleep == false && mIsAwake == false)
				setAwake(true);
		}

		void RigidBody::SetMass(const float mass)
		{
			mInverseMass = 1.0f / mass;
		}

		void RigidBody::SetVelocity(const float x, const float y, const float z)
		{
			mVelocity.x = x;
			mVelocity.y = y;
			mVelocity.z = z;
		}

		void RigidBody::SetAcceleration(const float x, const float y, const float z)
		{
			mAcceleration.x = x;
			mAcceleration.y = y;
			mAcceleration.z = z;
		}

		void RigidBody::SetRotation(const float x, const float y, const float z)
		{
			mRotation.x = x;
			mRotation.y = y;
			mRotation.z = z;
		}

		void RigidBody::SetOrientation(const float i, const float j, const float k, const float r)
		{
			mOrientation.x = i;
			mOrientation.y = j;
			mOrientation.z = k;
			mOrientation.w = r;

			DirectX::XMStoreFloat4(&mOrientation, DirectX::XMQuaternionNormalize(DirectX::XMLoadFloat4(&mOrientation)));
		}

		void RigidBody::SetDamping(const float linearDamping, const float angularDamping)
		{
			mLinearDamping = linearDamping;
			mAngularDamping = angularDamping;
		}

		void RigidBody::SetInertiaTensor(const DirectX::XMFLOAT4X4& InertiaTensor)
		{
			DirectX::XMMATRIX T = DirectX::XMLoadFloat4x4(&InertiaTensor);
			DirectX::XMStoreFloat4x4(&mInverseInertiaTensor, DirectX::XMMatrixInverse(&DirectX::XMMatrixDeterminant(T), T));
		}

		void RigidBody::AddVelocity(const float x, const float y, const float z)
		{
			mVelocity.x += x;
			mVelocity.y += y;
			mVelocity.z += z;
		}

		void RigidBody::AddRotation(const float x, const float y, const float z)
		{
			mRotation.x += x;
			mRotation.y += y;
			mRotation.z += z;
		}

		void RigidBody::GetPosition(DirectX::XMFLOAT4& Position)
		{
			Position.x = mPosition.x;
			Position.y = mPosition.y;
			Position.z = mPosition.z;
		}

		void RigidBody::GetRotation(DirectX::XMFLOAT4& Rotation)
		{
			Rotation.x = mRotation.x;
			Rotation.y = mRotation.y;
			Rotation.z = mRotation.z;
		}

		void RigidBody::GetVelocity(DirectX::XMFLOAT4& Velocity)
		{
			Velocity.x = mVelocity.x;
			Velocity.y = mVelocity.y;
			Velocity.z = mVelocity.z;
		}

		void RigidBody::GetOrientation(DirectX::XMFLOAT4& orientation)
		{
			orientation.x = mOrientation.x;
			orientation.y = mOrientation.y;
			orientation.z = mOrientation.z;
			orientation.w = mOrientation.w;
		}

		float RigidBody::GetInverseMass() const
		{
			return mInverseMass;
		}

		bool RigidBody::GetAwake() const
		{
			return mIsAwake;
		}

		void RigidBody::GetLastframeAcceleration(DirectX::XMFLOAT4& Acceleration)
		{
			Acceleration.x = mLastframeAcceleration.x;
			Acceleration.y = mLastframeAcceleration.y;
			Acceleration.z = mLastframeAcceleration.z;
		}

		void RigidBody::GetTransform(DirectX::XMFLOAT4X4& transformMatrix)
		{
			DirectX::XMMATRIX T = DirectX::XMLoadFloat4x4(&mTransformMatrix);
			DirectX::XMStoreFloat4x4(&transformMatrix, DirectX::XMMatrixTranspose(T));
		}

		void RigidBody::GetInverseInertiaTensorWorld(DirectX::XMFLOAT4X4& InertiaTensor)
		{
			InertiaTensor = mInverseInertiaTensorWorld;
		}
	}
}

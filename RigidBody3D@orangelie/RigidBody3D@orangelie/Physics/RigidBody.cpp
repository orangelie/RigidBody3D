#include "RigidBody.h"

namespace orangelie
{
	void RigidBody::CalculateDerivedData()
	{
		DirectX::XMVECTOR position = DirectX::XMLoadFloat4(&mPosition);
		DirectX::XMVECTOR quaternion = DirectX::XMLoadFloat4(&mOrientation);
		DirectX::XMVECTOR quatOrigin = DirectX::XMQuaternionIdentity();

		DirectX::XMMATRIX transformMat = DirectX::XMMatrixAffineTransformation(DirectX::XMVectorSet(1.0f, 1.0f, 1.0f, 0.0f), quatOrigin, quaternion, position);
		DirectX::XMStoreFloat4x4(&mTransformMatrix, DirectX::XMMatrixTranspose(transformMat));

		mTransformMatrix.m[0][3] = mPosition.x;
		mTransformMatrix.m[1][3] = mPosition.y;
		mTransformMatrix.m[2][3] = mPosition.z;


		DirectX::XMMATRIX A = DirectX::XMLoadFloat4x4(&mTransformMatrix);
		DirectX::XMMATRIX B = DirectX::XMLoadFloat4x4(&mInverseInertiaTensor);
		B = DirectX::XMMatrixMultiply(A, B);
		DirectX::XMStoreFloat4x4(&mInverseInertiaTensorWorld, DirectX::XMMatrixMultiply(B, DirectX::XMMatrixTranspose(A)));
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

		// Velocity Drag Damping;
		Utils::MathTool::Scalar(mVelocity, std::powf(mLinearDamping, dt));
		Utils::MathTool::Scalar(mRotation, std::powf(mAngularDamping, dt));

		// Position Update
		Utils::MathTool::AddScaledVector(mPosition, mVelocity, dt);
		Utils::MathTool::AddScaledQuaternion(mOrientation, angularAcceleration, dt);

		CalculateDerivedData();

		mForceAccum.x = mForceAccum.y = mForceAccum.z = 0.0f;
		mTorqueAccum.x = mTorqueAccum.y = mTorqueAccum.z = 0.0f;

		if (mCanSleep)
		{
			float motion =
				(mPosition.x * mPosition.x + mPosition.y * mPosition.y + mPosition.z * mPosition.z) +
				(mRotation.x * mRotation.x + mRotation.y * mRotation.y + mRotation.z * mRotation.z);
			float bias = std::powf(0.5f, dt);

			mMotion = mMotion * bias + motion * (1.0f - bias);

			if (mMotion < gSleepEpsilon)
				setAwake(false);
			else if (mMotion > 10.0f * gSleepEpsilon)
				mMotion = 10.0f * gSleepEpsilon;
		}
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

	void RigidBody::SetDamping(const float linearDamping, const float angularDamping)
	{
		mLinearDamping = linearDamping;
		mAngularDamping = angularDamping;
	}
}

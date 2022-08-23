#pragma once

#include "Contact.h"

using namespace DirectX;


namespace orangelie
{
	namespace Physics
	{
		struct CollisionData
		{
			Contact* contactArray = nullptr;
			Contact* contacts = nullptr;

			int contactsLeft = 0;
			unsigned contactCount = 0;

			float friction = 0.0f;
			float restitution = 0.0f;
			float tolerance = 0.0f;

			bool HasMoreContacts()
			{
				return (contactsLeft > 0);
			}

			void Reset(int maxContacts)
			{
				contactsLeft = maxContacts;
				contactCount = 0;

				contacts = contactArray;
			}

			void AddContact(int count)
			{
				contactsLeft -= count;
				contactCount += count;

				contacts += count;
			}
		};


		class CollisionPrimitive
		{
		public:
			friend class IntersectTests;
			friend class CollisionDetector;

			RigidBody* mBody;
			DirectX::XMFLOAT3X4 mOffset = Utils::MatrixIdentity3();

			void CalculateInternals();

			DirectX::XMVECTOR GetAxis(unsigned index) const
			{
				return DirectX::XMVectorSet(mTransform.m[0][index], mTransform.m[1][index], mTransform.m[2][index], 0.0f);
			}

			const DirectX::XMFLOAT3X4& GetTransform() const
			{
				return mTransform;
			}

		protected:
			DirectX::XMFLOAT3X4 mTransform = Utils::MatrixIdentity3();

		};

		class CollisionBox : public CollisionPrimitive
		{
		public:
			DirectX::XMFLOAT4 mHalfSize;

		};

		class CollisionSphere : public CollisionPrimitive
		{
		public:
			float mRadius;

		};

		class CollisionPlane
		{
		public:
			DirectX::XMFLOAT4 mDirection;
			float mOffset;

		};


		class IntersectTests
		{
		public:
			inline static bool SphereAndSphere(CollisionSphere& one, CollisionSphere& two);
			inline static bool BoxAndBox(CollisionBox& one, CollisionBox& two);

		};

		class CollisionDetector
		{
		public:
			static unsigned SphereAndSphere(CollisionSphere& one, CollisionSphere& two, CollisionData* data);
			static unsigned BoxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data);

		};


		inline static DirectX::XMVECTOR ContactPoint(
			DirectX::XMVECTOR pOne, DirectX::XMVECTOR dOne, float oneSize,
			DirectX::XMVECTOR pTwo, DirectX::XMVECTOR dTwo, float twoSize, bool useOne);
		inline static float VectorGet(const DirectX::XMFLOAT4& vec, unsigned index);
		inline static void VectorSetConvert(DirectX::XMFLOAT4& vec, unsigned index);
		inline static void VectorSetZero(DirectX::XMFLOAT4& vec, unsigned index);
		inline static void FillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data, const DirectX::XMVECTOR& toCentre, float pen, unsigned best);
		inline static float TransformToAxis(const CollisionBox& box, DirectX::XMVECTOR axis);
		inline static float PenetrationAxis(const CollisionBox& one, const CollisionBox& two, const DirectX::XMVECTOR& axis, const DirectX::XMVECTOR& toCentre);
		inline static bool TryAxis(const CollisionBox& one, const CollisionBox& two, DirectX::XMVECTOR axis, DirectX::FXMVECTOR& toCentre,
			unsigned index, float& smallestPenetraion, unsigned& smallestCase);
	}
}
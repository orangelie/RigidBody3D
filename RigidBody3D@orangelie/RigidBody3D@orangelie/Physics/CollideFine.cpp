#include "CollideFine.h"


namespace orangelie
{
	namespace Physics
	{
		void CollisionPrimitive::CalculateInternals()
		{
			DirectX::XMFLOAT4X4 transform = {};
			mBody->GetTransform(transform);

			DirectX::XMStoreFloat4x4(&mTransform, DirectX::XMLoadFloat4x4(&transform) * DirectX::XMLoadFloat4x4(&mOffset));
		}

		inline
		bool IntersectTests::SphereAndSphere(CollisionSphere& one, CollisionSphere& two)
		{
			XMVECTOR v = one.GetAxis(3) - two.GetAxis(3);

			return XMVectorGetX(XMVector3LengthSq(v)) < (one.mRadius + two.mRadius) * (one.mRadius + two.mRadius);
		}

		inline
		bool IntersectTests::BoxAndBox(CollisionBox& one, CollisionBox& two)
		{


			return true;
		}

		unsigned CollisionDetector::SphereAndSphere(CollisionSphere& one, CollisionSphere& two, CollisionData* data)
		{
			XMVECTOR pos1 = one.GetAxis(3);
			XMVECTOR pos2 = two.GetAxis(3);
			XMVECTOR midline = pos1 - pos2;
			XMVECTOR norm = XMVector3Normalize(midline);
			float length = XMVectorGetX(XMVector3Length(midline));

			if (length <= 0.0f || length >= (one.mRadius + two.mRadius))
			{
				return 0;
			}

			Contact* contact = data->contacts;
			XMStoreFloat4(&contact->mContactNormal, norm);
			XMStoreFloat4(&contact->mContactPoint, pos1 + midline * 0.5f);
			contact->mPenetration = (one.mRadius + two.mRadius) - length;
			contact->SetBodyData(one.mBody, two.mBody, data->friction, data->restitution);

			data->AddContact(1);

			return 1;
		}

#define OVERLAP_AXIS(axis, index) if(!TryAxis(one, two, (axis), toCentre, (index), pen, best)) return 0;
		unsigned CollisionDetector::BoxAndBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data)
		{
			XMVECTOR toCentre = two.GetAxis(3) - one.GetAxis(3);

			float pen = FLT_MAX;
			unsigned best = 0xffffff;

			OVERLAP_AXIS(one.GetAxis(0), 0);

			// MessageBoxA(0, std::to_string(two.GetTransform().m[0][3]).c_str(), "", MB_OK);

			OVERLAP_AXIS(one.GetAxis(1), 1);
			OVERLAP_AXIS(one.GetAxis(2), 2);

			OVERLAP_AXIS(two.GetAxis(0), 3);
			OVERLAP_AXIS(two.GetAxis(1), 4);
			OVERLAP_AXIS(two.GetAxis(2), 5);

			unsigned bestSingleAxis = best;

			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(0), two.GetAxis(0)), 6);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(0), two.GetAxis(1)), 7);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(0), two.GetAxis(2)), 8);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(1), two.GetAxis(0)), 9);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(1), two.GetAxis(1)), 10);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(1), two.GetAxis(2)), 11);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(2), two.GetAxis(0)), 12);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(2), two.GetAxis(1)), 13);
			OVERLAP_AXIS(XMVector3Cross(one.GetAxis(2), two.GetAxis(2)), 14);

			assert(best != 0xffffff);

			if (best < 3)
			{
				FillPointFaceBoxBox(one, two, data, toCentre, pen, best);
				data->AddContact(1);
				return 1;
			}
			else if (best < 6)
			{
				FillPointFaceBoxBox(one, two, data, toCentre * -1.0f, pen, best - 3);
				data->AddContact(1);
				return 1;
			}
			else
			{
				best -= 6;
				unsigned oneAxisIndex = best / 6;
				unsigned twoAxisIndex = best % 6;
				XMVECTOR oneAxis = one.GetAxis(oneAxisIndex);
				XMVECTOR twoAxis = two.GetAxis(twoAxisIndex);
				XMVECTOR axis = XMVector3Cross(oneAxis, twoAxis);
				axis = XMVector3Normalize(axis);

				if (XMVectorGetX(XMVector3Dot(axis, toCentre)) > 0.0f)
				{
					axis = axis * -1.0f;
				}


				XMFLOAT4 ptOnOneEdgeF = one.mHalfSize;
				XMFLOAT4 ptOnTwoEdgeF = two.mHalfSize;

				for (unsigned i = 0; i < 3; ++i)
				{
					if (i == oneAxisIndex)
						VectorSetZero(ptOnOneEdgeF, i);
					else if (XMVectorGetX(XMVector3Dot(one.GetAxis(i), axis)) > 0.0f)
						VectorSetConvert(ptOnOneEdgeF, i);

					if (i == twoAxisIndex)
						VectorSetZero(ptOnTwoEdgeF, i);
					else if (XMVectorGetX(XMVector3Dot(two.GetAxis(i), axis)) < 0.0f)
						VectorSetConvert(ptOnTwoEdgeF, i);
				}

				XMVECTOR ptOnOneEdge = XMLoadFloat4(&ptOnOneEdgeF);
				XMVECTOR ptOnTwoEdge = XMLoadFloat4(&ptOnTwoEdgeF);

				ptOnOneEdge = XMVector3TransformCoord(ptOnOneEdge, XMLoadFloat4x4(&one.GetTransform()));
				ptOnTwoEdge = XMVector3TransformCoord(ptOnTwoEdge, XMLoadFloat4x4(&two.GetTransform()));

				XMVECTOR vertex = ContactPoint(
					ptOnOneEdge, oneAxis, VectorGet(one.mHalfSize, oneAxisIndex),
					ptOnTwoEdge, twoAxis, VectorGet(two.mHalfSize, twoAxisIndex), bestSingleAxis > 2);


				Contact* contact = data->contacts;
				XMStoreFloat4(&contact->mContactNormal, axis);
				XMStoreFloat4(&contact->mContactPoint, vertex);
				contact->mPenetration = pen;

				contact->SetBodyData(one.mBody, two.mBody, data->friction, data->restitution);
				data->AddContact(1);
			}

			return 1;
		}
#undef OVERLAP_AXIS

		inline static DirectX::XMVECTOR ContactPoint(
			DirectX::XMVECTOR pOne, DirectX::XMVECTOR dOne, float oneSize,
			DirectX::XMVECTOR pTwo, DirectX::XMVECTOR dTwo, float twoSize, bool useOne)
		{

			float smOne = XMVectorGetX(XMVector3LengthSq(dOne));
			float smTwo = XMVectorGetX(XMVector3LengthSq(dTwo));
			float dpOneTwo = XMVectorGetX(XMVector3Dot(dOne, dTwo));

			XMVECTOR toSt = pOne - pTwo;
			float dpStaOne = XMVectorGetX(XMVector3Dot(dOne, toSt));
			float dpStaTwo = XMVectorGetX(XMVector3Dot(dTwo, toSt));

			float denom = smOne * smTwo - dpOneTwo * dpOneTwo;

			if (std::fabsf(denom) < 0.0001f)
				return useOne ? pOne : pTwo;


			float mua = (dpOneTwo * dpStaTwo - smTwo * dpStaOne) / denom;
			float mub = (smOne * dpStaTwo - dpOneTwo * dpStaOne) / denom;

			if (mua > oneSize || mua < -oneSize || mub > twoSize || mub < -twoSize)
			{
				return useOne ? pOne : pTwo;
			}
			else
			{
				XMVECTOR cOne = pOne + dOne * mua;
				XMVECTOR cTwo = pTwo + dTwo * mub;

				return 0.5f * cOne + 0.5f * cTwo;
			}
		}

		inline static float VectorGet(const DirectX::XMFLOAT4& vec, unsigned index)
		{
			if (index == 0)
				return vec.x;
			else if (index == 1)
				return vec.y;

			return vec.z;
		}

		inline static void VectorSetConvert(DirectX::XMFLOAT4& vec, unsigned index)
		{
			if (index == 0)
				vec.x = -vec.x;
			else if (index == 1)
				vec.y = -vec.y;
			else if (index == 2)
				vec.z = -vec.z;
		}

		inline static void VectorSetZero(DirectX::XMFLOAT4& vec, unsigned index)
		{
			if (index == 0)
				vec.x = 0.0f;
			else if (index == 1)
				vec.y = 0.0f;
			else if (index == 2)
				vec.z = 0.0f;
			else if (index == 3)
				vec.w = 0.0f;
		}

		inline static void FillPointFaceBoxBox(const CollisionBox& one, const CollisionBox& two, CollisionData* data,
			const DirectX::XMVECTOR& toCentre, float pen, unsigned best)
		{
			Contact* contact = data->contacts;

			XMVECTOR normal = one.GetAxis(best);
			if (XMVectorGetX(XMVector3Dot(one.GetAxis(best), toCentre)) > 0.0f)
			{
				normal *= -1.0f;
			}

			XMFLOAT4 vertexF = two.mHalfSize;
			if (XMVectorGetX(XMVector3Dot(two.GetAxis(0), normal)) < 0.0f)
				vertexF.x = -vertexF.x;
			if (XMVectorGetX(XMVector3Dot(two.GetAxis(1), normal)) < 0.0f)
				vertexF.y = -vertexF.y;
			if (XMVectorGetX(XMVector3Dot(two.GetAxis(2), normal)) < 0.0f)
				vertexF.z = -vertexF.z;

			XMStoreFloat4(&contact->mContactNormal, normal);
			XMStoreFloat4(&contact->mContactPoint, XMVector3TransformCoord(XMLoadFloat4(&vertexF), XMLoadFloat4x4(&two.GetTransform())));
			contact->mPenetration = pen;

			contact->SetBodyData(one.mBody, two.mBody, data->friction, data->restitution);
		}

		inline static float TransformToAxis(const CollisionBox& box, DirectX::XMVECTOR axis)
		{
			return
				box.mHalfSize.x * (std::fabsf(XMVectorGetX(XMVector3Dot(axis, box.GetAxis(0))))) +
				box.mHalfSize.y * (std::fabsf(XMVectorGetX(XMVector3Dot(axis, box.GetAxis(1))))) +
				box.mHalfSize.z * (std::fabsf(XMVectorGetX(XMVector3Dot(axis, box.GetAxis(2)))));
		}

		inline static float PenetrationAxis(const CollisionBox& one, const CollisionBox& two, const DirectX::XMVECTOR& axis, const DirectX::XMVECTOR& toCentre)
		{
			float oneAxis = TransformToAxis(one, axis);
			float twoAxis = TransformToAxis(two, axis);

			float dist = (std::fabsf(XMVectorGetX(XMVector3Dot(toCentre, axis))));

			return oneAxis + twoAxis - dist;
		}

		inline static bool TryAxis(const CollisionBox& one, const CollisionBox& two, DirectX::XMVECTOR axis, DirectX::FXMVECTOR& toCentre,
			unsigned index, float& smallestPenetraion, unsigned& smallestCase)
		{
			if (XMVectorGetX(XMVector3LengthSq(axis)) < 0.0001f)
				return true;
			axis = XMVector3Normalize(axis);

			float penetration = PenetrationAxis(one, two, axis, toCentre);

			if (penetration < 0.0f)
				return false;
			if (penetration < smallestPenetraion)
			{
				smallestPenetraion = penetration;
				smallestCase = index;
			}

			return true;
		}
	}
}

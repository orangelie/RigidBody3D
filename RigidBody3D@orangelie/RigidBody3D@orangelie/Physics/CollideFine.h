#pragma once

#include "Contact.h"

namespace orangelie
{
	namespace Physics
	{
		struct CollisionData
		{
			Contact* contactArray;
			Contact* contacts;

			int contactsLeft;
			unsigned contactCount;

			float friction;
			float restitution;
			float tolerance;

			bool HasMoreContacts()
			{
				return (contactsLeft > 0);
			}

			void reset(int maxContacts)
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

		class CollisionDetector
		{
		public:


		};
	}
}
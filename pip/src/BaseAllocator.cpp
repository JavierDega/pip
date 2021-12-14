#include "BaseAllocator.h"

size_t BaseAllocator::GetBodyByteSize(Rigidbody* rb)
{
	assert(rb);
	switch (rb->m_bodyType)
	{
	case BodyType::Circle:
	{
		return sizeof(Circle);
		break;
	}
	case BodyType::Capsule:
	{
		return sizeof(Capsule);
		break;
	}
	case BodyType::Obb:
	{
		return sizeof(OrientedBox);
		break;
	}
	default:
		break;
	}
	return size_t();
}

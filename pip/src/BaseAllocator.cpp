#include "BaseAllocator.h"

#include <assert.h>

#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"

BaseAllocator::BaseAllocator()
{
}


BaseAllocator::~BaseAllocator()
{
}

size_t BaseAllocator::GetBodyByteSize(Rigidbody* rb)
{
	assert(rb);
	switch (rb->GetBodyType())
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

#include "DefaultAllocator.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"
#include <assert.h>
#include <string.h>

using namespace std;

DefaultAllocator::DefaultAllocator(size_t poolSize)
	: m_bodyCount(0)
{
	if (poolSize > 0)
	{
		CreatePool(poolSize);
	}
}


DefaultAllocator::~DefaultAllocator()
{
	DestroyPool();
}

void DefaultAllocator::CreatePool(size_t size)
{
	m_pool.start = (char*)(malloc(size));
	m_pool.next = m_pool.start;
	m_pool.end = m_pool.start + size;
}

void DefaultAllocator::DestroyPool()
{
	DestroyAllBodies();
	free(m_pool.start);
	m_pool.start = nullptr;
	m_pool.next = nullptr;
	m_pool.end = nullptr;
}

void* DefaultAllocator::AllocateBody( size_t length, Handle& handle)
{
	//Asks for a linear slot of that size from the pool and return void *
	assert( length <= AvailableInPool());
	size_t objIdx = m_objectToMappingIdx.size();
	// Try to recycle a gap in the mapping list
	for (size_t i = 0; i < m_mappings.size(); i++)
	{
		if (!m_mappings[i].active)
		{
			m_objectToMappingIdx.push_back(i);

			m_mappings[i].active = true;
			m_mappings[i].generation++;
			m_mappings[i].idx = objIdx;
			
			handle.idx = i;
			handle.generation = m_mappings[i].generation;
			char* ret = m_pool.next;
			m_pool.next += length;
			return (void*)ret;
		}
	}
	// Otherwise use a new mapping idx
	m_objectToMappingIdx.push_back(m_mappings.size());

	handle.idx = m_mappings.size();
	handle.generation = 0;

	Idx idx = Idx(true, objIdx, 0);
	m_mappings.push_back(idx);

	//Allocate memory and move pool pointer
	char* ret = m_pool.next;
	m_pool.next += length;
	return (void*)ret;
}

void DefaultAllocator::DestroyAllBodies()
{
	memset(m_pool.start, 0, m_pool.end - m_pool.start);//#Profile memleak
	m_pool.next = m_pool.start;
	m_mappings.clear();
	m_objectToMappingIdx.clear();
}
//Bytes available
size_t DefaultAllocator::AvailableInPool()
{
	return m_pool.end - m_pool.next;
}

Rigidbody* DefaultAllocator::GetFirstBody()
{
	//Returns null if pool uninitted
	return (Rigidbody*)m_pool.start;
}

Rigidbody* DefaultAllocator::GetNextBody(Rigidbody* prev)
{
	assert(AvailableInPool() > sizeof(Rigidbody));
	char* charP = (char*)prev;
	char* charPNext = charP;
	switch (prev->m_bodyType)
	{
		case BodyType::Circle:
		{
			charPNext += sizeof(Circle);
			break;
		}
		case BodyType::Capsule:
		{
			charPNext += sizeof(Capsule);
			break;
		}
		case BodyType::Obb:
		{
			charPNext += sizeof(OrientedBox);
			break;
		}
		default:
			break;
	}
	if (charPNext == m_pool.next) return nullptr;
	return (Rigidbody*)charPNext;
}

Rigidbody* DefaultAllocator::GetBody(Handle handle)
{
	if (!IsHandleValid(handle)) return nullptr;
	Idx i = m_mappings[handle.idx];
	return GetBodyAt(i.idx);
}

Rigidbody* DefaultAllocator::GetBodyAt(size_t i)
{
	//#Using pool mappings, could rapidly figure out memory offset from pool's m_start
	//by comparing bodyTypes of all PoolIdx's
	assert(i < m_objectToMappingIdx.size());
	size_t curIdx = 0;
	Rigidbody* rb = (Rigidbody*)m_pool.start;
	for (size_t curIdx = 0; curIdx < i; curIdx++)
	{
		assert(rb);
		if (rb == nullptr)
		{
			cout << "PiP Error: Trying to get body index bigger than poolSize" << endl;
			return nullptr;
		}
		rb = GetNextBody(rb);
	}
	return rb;
}

Rigidbody* DefaultAllocator::GetLastBodyOfType(BodyType bodyType)
{
	Rigidbody* lastBodyOfType = nullptr;
	for (Rigidbody* rb = GetFirstBody(); rb != nullptr; rb = GetNextBody(rb))
	{
		if (rb->m_bodyType == bodyType)
		{
			lastBodyOfType = rb;
		}
	}

	return lastBodyOfType;
}

void DefaultAllocator::DestroyBody(Handle handle)
{
	if (!IsHandleValid(handle))
	{
		cout << "PiP Warning: DestroyBody::Handle invalid" << endl;
		return;
	}

	//Since pools are multiobjects, we need to swap and pop with last object of same body type
	//Loop till we find first object of same bodyType, and check whether its the same object.
	size_t objIdx = m_mappings[handle.idx].idx;
	BodyType bodyType = GetBody(handle)->m_bodyType;
	Rigidbody* lastMatchingShape = GetLastBodyOfType(bodyType);//Swap and pop with last object of same shape, displace every object following

	//Need to update, swap and pop mappings and object pool, deactivate mapping,..
	/*  size_t obj_idx = mapping[handle.idx].idx;
        Object<T>& last_obj = objects.back();

        // Point last object's mapping idx to point at the object that will be destroyed
        mapping[last_obj.mapping_idx].idx = obj_idx;

        // Swap the last object with the object to be destroyed, then pop the vec
        objects[obj_idx] = last_obj;
        objects.pop_back();

        // Set the mapping to be inactive for the object that was destroyed
        mapping[handle.idx].active = false;*/
}

bool DefaultAllocator::IsHandleValid(Handle handle)
{
	return handle.idx < m_mappings.size() && m_mappings[handle.idx].active && handle.generation == m_mappings[handle.idx].generation;
}

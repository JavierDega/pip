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

void* DefaultAllocator::AllocateBody(size_t length, Handle& handle)
{
	//Asks for a linear slot of that size from the pool and return void *
	assert(length <= AvailableInPool());
	// Try to recycle a gap in the mapping list
	for (size_t i = 0; i < m_mapping.size(); i++)
	{
		if (!m_mapping[i].active)
		{
			m_mapping[i].active = true;
			m_mapping[i].generation++;
			m_mapping[i].poolIdx = m_bodyCount;
			
			handle.mappingIdx = i;
			handle.generation = m_mapping[i].generation;
		}
	}
	// Otherwise use a new mapping idx
	handle.mappingIdx = m_mapping.size();
	handle.generation = 0;

	Idx mapIdx = Idx(true, m_bodyCount, 0);
	m_mapping.push_back(mapIdx);
	//Allocate memory and move pool pointer
	char* ret = m_pool.next;
	m_pool.next += length;
	m_bodyCount++;
	return (void*)ret;
}

void DefaultAllocator::DestroyAllBodies()
{
	memset(m_pool.start, 0, m_pool.end - m_pool.start);//#Profile memleak
	m_pool.next = m_pool.start;
	m_bodyCount = 0;
	m_mapping.clear();
}

size_t DefaultAllocator::AvailableInPool()
{
	return m_pool.end - m_pool.next;
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
			charPNext += sizeof(Obb);
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
	Idx i = m_mapping[handle.mappingIdx];
	return GetBodyAt(i.poolIdx);
}

Rigidbody* DefaultAllocator::GetBodyAt(size_t i)
{
	//#Do downcasting or embed body type enum in Rigidbody class for efficiency
	size_t curIdx = 0;
	Rigidbody* rb = (Rigidbody*)m_pool.start;
	for (size_t curIdx = 0; curIdx < i; curIdx++)
	{
		if (rb == nullptr)
		{
			cout << "PiP Error: Trying to get body index bigger than poolSize" << endl;
			return nullptr;
		}
		rb = GetNextBody(rb);
	}
	return rb;
}

void DefaultAllocator::DestroyBody(Handle handle)
{
	/*if (!is_valid(handle))
		return;

	size_t obj_idx = mapping[handle.idx].idx;
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
	return handle.mappingIdx < m_mapping.size() && m_mapping[handle.mappingIdx].active && handle.generation == m_mapping[handle.mappingIdx].generation;
}

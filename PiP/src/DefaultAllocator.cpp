#include "DefaultAllocator.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"
#include <assert.h>
#include <string.h>

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
	char* charPNext;
	if (Circle* circle = dynamic_cast<Circle*>(prev)) {
		charPNext = charP + sizeof(Circle);
	}
	else if (Capsule* capsule = dynamic_cast<Capsule*>(prev)) {
		charPNext = charP + sizeof(Capsule);
	}
	else if (OrientedBox* obb = dynamic_cast<OrientedBox*>(prev)) {
		charPNext = charP + sizeof(OrientedBox);
	}
	if (charPNext == m_pool.next) return nullptr;
	return (Rigidbody*)charPNext;
}

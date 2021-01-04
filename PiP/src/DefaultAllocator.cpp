#include "DefaultAllocator.h"
#include "Circle.h"
#include "Capsule.h"
#include "OrientedBox.h"
#include <assert.h>
#include <string.h>

using namespace std;

DefaultAllocator::DefaultAllocator(size_t poolSize)
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

void* DefaultAllocator::AllocateBody( BodyType bodyType, Handle& handle)
{
	//Asks for a linear slot of that size from the pool and return void *
	size_t bodyLength;
	switch (bodyType)
	{
		case BodyType::E_Circle:
		{
			bodyLength = sizeof(Circle);
			break;
		}
		case BodyType::E_Capsule:
		{
			bodyLength = sizeof(Capsule);
			break;
		}
		case BodyType::E_Obb:
		{
			bodyLength = sizeof(OrientedBox);
			break;
		}
		default:
			break;
	}

	assert( bodyLength <= AvailableInPool());

	size_t objIdx = m_poolMappings.size();
	// Try to recycle a gap in the mapping list
	for (size_t i = 0; i < m_mappings.size(); i++)
	{
		if (!m_mappings[i].active)
		{
			m_poolMappings.push_back(PoolIdx(bodyType, i));
			
			m_mappings[i].active = true;
			m_mappings[i].generation++;
			m_mappings[i].poolIdx = objIdx;
			
			handle.mappingIdx = i;
			handle.generation = m_mappings[i].generation;
		}
	}
	// Otherwise use a new mapping idx
	m_poolMappings.push_back(PoolIdx(bodyType, m_mappings.size()));
	
	handle.mappingIdx = m_mappings.size();
	handle.generation = 0;

	MappingIdx mapIdx = MappingIdx(true, objIdx, 0);
	m_mappings.push_back(mapIdx);
	
	//Allocate memory and move pool pointer
	char* ret = m_pool.next;
	m_pool.next += bodyLength;
	return (void*)ret;
}

void DefaultAllocator::DestroyAllBodies()
{
	memset(m_pool.start, 0, m_pool.end - m_pool.start);//#Profile memleak
	m_pool.next = m_pool.start;
	m_mappings.clear();
	m_poolMappings.clear();
}
//Bytes available
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
		case BodyType::E_Circle:
		{
			charPNext += sizeof(Circle);
			break;
		}
		case BodyType::E_Capsule:
		{
			charPNext += sizeof(Capsule);
			break;
		}
		case BodyType::E_Obb:
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
	MappingIdx i = m_mappings[handle.mappingIdx];
	return GetBodyAt(i.poolIdx);
}

Rigidbody* DefaultAllocator::GetBodyAt(size_t i)
{
	//#Using pool mappings, could rapidly figure out memory offset from pool's m_start
	//by comparing bodyTypes of all PoolIdx's
	assert(i < m_poolMappings.size());
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
	if (!IsHandleValid(handle))
	{
		cout << "PiP Warning: DestroyBody::Handle invalid" << endl;
		return;
	}

	//Since pools are multiobjects, we need to swap and pop with last object of same body type
	//Loop from end till we find first object of same bodyType, and check whether its the same object.
	size_t objIdx = m_mappings[handle.mappingIdx].poolIdx;
	BodyType bodyType = GetBody(handle)->m_bodyType;
	for (int i = m_poolMappings.size() - 1; i >= 0; i--)
	{
		PoolIdx poolIdx = m_poolMappings[i];
		if (poolIdx.bodyType == bodyType)
		{
			//Check its not same object
			if (poolIdx.mappingIdx != handle.mappingIdx)
			{
				//Swap and pop
				//Point last object's mapping idx to point at the object that will be destroyed
				m_mappings[poolIdx.mappingIdx].poolIdx = objIdx;

				// Swap the last object with the object to be destroyed, pop, then offset remaining pool objects
				m_poolMappings[objIdx] = poolIdx;
				m_poolMappings.erase(m_poolMappings.begin() + i);
			}
		}
	}
	//Need to update, swap and pop mappings and object pool, deactivate mapping,..
}

bool DefaultAllocator::IsHandleValid(Handle handle)
{
	return handle.mappingIdx < m_mappings.size() && m_mappings[handle.mappingIdx].active && handle.generation == m_mappings[handle.mappingIdx].generation;
}

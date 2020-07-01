#include "DefaultAllocator.h"
#include <assert.h>
#include <string.h>

DefaultAllocator::DefaultAllocator()
{
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

void* DefaultAllocator::AllocateBody(size_t length)
{
	//asks for a linear slot of that size from the pool and return void *
	assert(length <= AvailableInPool());
	char* ret = m_pool.next;
	m_pool.next += length;
	return (void*)ret;
}

void DefaultAllocator::DestroyAllBodies()
{
	memset(m_pool.start, 0, m_pool.end - m_pool.start);//Profile memleak
	m_pool.next = m_pool.start;
}

size_t DefaultAllocator::AvailableInPool()
{
	return m_pool.end - m_pool.next;
}

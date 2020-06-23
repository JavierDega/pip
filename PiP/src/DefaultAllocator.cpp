#include "DefaultAllocator.h"
#include <assert.h>

DefaultAllocator::DefaultAllocator()
{
}


DefaultAllocator::~DefaultAllocator()
{
}
void DefaultAllocator::CreatePool(size_t size)
{
	m_pool.start = (char*)(malloc(size));
	m_pool.next = m_pool.start;
	m_pool.end = m_pool.start + size;
}

void* DefaultAllocator::AllocateBody(size_t length)
{
	//asks for a linear slot of that size from the pool and return void *
	assert(length <= AvailableInPool());
	char* ret = m_pool.next;
	m_pool.next += length;
	return (void*)ret;
}

size_t DefaultAllocator::AvailableInPool()
{
	return m_pool.end - m_pool.next;
}

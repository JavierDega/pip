#pragma once
#include <stdlib.h>

#include "Rigidbody.h"

class DefaultAllocator
{
public:
	DefaultAllocator(size_t poolSize = 0);
	~DefaultAllocator();

	void CreatePool(size_t size);
	void DestroyPool();//Profile whether free deallocates whole pool
	void * AllocateBody(size_t length);
	void DestroyAllBodies();//Won't call destructors
	size_t AvailableInPool();
	Rigidbody* GetNextBody(Rigidbody* prev);

	typedef struct PoolStr
	{
		char* start;
		char* next;//Where to allocate next
		char* end;
	} Pool;

	Pool m_pool;
};
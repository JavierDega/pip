#pragma once
#include <stdlib.h>

class DefaultAllocator
{
public:
	DefaultAllocator();
	~DefaultAllocator();

	void CreatePool(size_t size);
	void DestroyPool();//Profile whether free deallocates whole pool
	void * AllocateBody(size_t length);
	void DestroyAllBodies();//Won't call destructors
	size_t AvailableInPool();

	typedef struct PoolStr
	{
		char* start;
		char* next;
		char* end;
	} Pool;

	Pool m_pool;
	/*Pool * Pool_Create(size_t size) {
		Pool *p = (Pool*)malloc(size + sizeof(Pool));
		p->next = (char*)&p[1];
		p->end = p->next + size;
		return p;
	}

	void Pool_Destroy(Pool *p) {
		free(p);
	}
	}*/
};
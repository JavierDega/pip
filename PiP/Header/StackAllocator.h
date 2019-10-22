#pragma once
#include "stdlib.h"

class StackAllocator
{
public:
	StackAllocator();
	~StackAllocator();

private:
	typedef struct PoolStr
	{
		char * next;
		char * end;
	} Pool;

	Pool * Pool_Create(size_t size) {
		Pool *p = (Pool*)malloc(size + sizeof(Pool));
		p->next = (char*)&p[1];
		p->end = p->next + size;
		return p;
	}
	void Pool_Destroy(Pool *p) {
		free(p);
	}

	size_t Pool_Available(Pool *p) {
		return p->end - p->next;
	}
};
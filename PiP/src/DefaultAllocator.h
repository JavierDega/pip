#pragma once
#include <stdlib.h>

#include "Rigidbody.h"

struct Handle
{
    Handle(size_t idx = 0, uint64_t generation = 0)
    {
        this->mappingIdx = idx;
        this->generation = generation;
    }
    size_t mappingIdx;
    uint64_t generation;
};

struct Idx
{
    Idx(bool active, size_t idx, uint64_t generation)
    {
        this->active = active;
        this->poolIdx = idx;
        this->generation = generation;
    }

    bool active;
    size_t poolIdx;
    uint64_t generation;
};

class DefaultAllocator
{
public:
	DefaultAllocator(size_t poolSize = 0);
	~DefaultAllocator();

	void CreatePool(size_t size);
	void DestroyPool();//Profile whether free deallocates whole pool
	void * AllocateBody(size_t length, Handle& handle);
	void DestroyAllBodies();//Won't call destructors
	size_t AvailableInPool();
	Rigidbody* GetNextBody(Rigidbody* prev);
    Rigidbody* GetBody(Handle handle);
    Rigidbody* GetBodyAt(size_t i);
    void DestroyBody(Handle handle);
    bool IsHandleValid(Handle handle);

	typedef struct PoolStr
	{
		char* start;
		char* next;//Where to allocate next
		char* end;
	} Pool;

	Pool m_pool;
    std::vector<Idx> m_mapping;
    unsigned int m_bodyCount;
};

/*
int main()
{
    Entities<int> entities;
    Handle first = entities.add_entity(1);
    Handle second = entities.add_entity(2);
    Handle third = entities.add_entity(3);

    entities.destroy(second);

    int* p_first = entities.get_entity(first);
    int* p_second = entities.get_entity(second);
    int* p_third = entities.get_entity(third);

    assert(*p_first == 1);
    assert(p_second == nullptr);
    assert(*p_third == 3);
}
*/
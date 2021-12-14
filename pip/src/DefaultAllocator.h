#pragma once

#include <stdlib.h>
#include "BaseAllocator.h"

struct Idx
{
    Idx(bool active, size_t i, uint64_t generation)
    {
        this->active = active;
        this->idx = i;
        this->generation = generation;
    }

    bool active;
    size_t idx;
    uint64_t generation;
};

struct Pool
{
    Pool()
    {
        start = nullptr;
        next = nullptr;
        end = nullptr;
    }
    char* start;
    char* next;//Where to allocate next
    char* end;
};

class DefaultAllocator : BaseAllocator
{
public:
	DefaultAllocator(size_t poolSize = 0);
	~DefaultAllocator();
    //BaseAllocator
	virtual void* AllocateBody(size_t length, Handle& handle) override;
	virtual void DestroyAllBodies() override;//Won't call destructors
    virtual void DestroyBody(Handle handle) override;
    virtual Rigidbody* GetFirstBody() override;
	virtual Rigidbody* GetNextBody(Rigidbody* prev) override;
    Rigidbody* GetBody(Handle handle) override;
    Rigidbody* GetBodyAt(size_t i) override;
    virtual bool IsHandleValid(Handle handle) override;
private:
	void CreatePool(size_t size);
	void DestroyPool();//Profile whether free deallocates whole pool
	size_t AvailableInPool();
    Rigidbody* GetLastBodyOfType(BodyType bodyType, int& idx);
    void DestroyBodyFromPool(Rigidbody* bodyToDestroy);//Realigns pool
public:
private:
	Pool m_pool;
    std::vector<Idx> m_mappings;//Maps reusable object list to linear object pool.
    std::vector<size_t> m_objectToMappingIdx;//Maps object idx in the pool to their mapping idx
};

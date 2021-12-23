#pragma once

#include "Rigidbody.h"

struct Handle
{
    Handle(size_t i = 0, uint64_t generation = 0)
    {
        this->idx = i;
        this->generation = generation;
    }
    size_t idx;
    uint64_t generation;
};

class BaseAllocator
{
public:
	BaseAllocator();
	virtual ~BaseAllocator();
    size_t GetBodyByteSize(Rigidbody* rb);
	virtual void* AllocateBody(size_t length, Handle& handle) = 0;
	virtual void DestroyAllBodies() = 0;//Won't call destructors
    virtual void DestroyBody(Handle handle) = 0;
    virtual Rigidbody* GetFirstBody() = 0;
	virtual Rigidbody* GetNextBody(Rigidbody* prev) = 0;
    virtual Rigidbody* GetBody(Handle handle) = 0;
    virtual Rigidbody* GetBodyAt(size_t i) = 0;
    virtual bool IsHandleValid(Handle handle) = 0;
public:
private:
};

/*
#include <vector>
#include <assert.h>
#include <stdint.h>

struct Handle
{
    size_t idx;
    uint64_t generation;
};

struct Idx
{
    bool active;
    size_t idx;
    uint64_t generation;
};

template <typename T>
struct Object
{
    size_t mapping_idx;
    T data;
};

template <typename T>
struct Entities
{
    std::vector<Idx> mapping;
    std::vector<Object<T>> objects;

    Handle add_entity(T entity)
    {
        size_t obj_idx = objects.size();

        // Try to recycle a gap in the mapping list
        for (size_t i = 0; i < mapping.size(); i++)
        {
            if (!mapping[i].active)
            {
                Object<T> object;
                object.mapping_idx = i;
                object.data = entity;
                objects.push_back(object);

                mapping[i].active = true;
                mapping[i].generation += 1;
                mapping[i].idx = obj_idx;

                Handle handle;
                handle.idx = i;
                handle.generation = mapping[i].generation;
                return handle;
            }
        }

        // Otherwise use a new mapping idx
        Object<T> object;
        object.mapping_idx = mapping.size();
        object.data = entity;
        objects.push_back(object);

        Handle handle;
        handle.idx = mapping.size();
        handle.generation = 0;

        Idx map_idx;
        map_idx.active = true;
        map_idx.idx = obj_idx;
        map_idx.generation = 0;
        mapping.push_back(map_idx);

        return handle;
    }

    T* get_entity(Handle handle)
    {
        if (!is_valid(handle))
            return nullptr;

        return &objects[mapping[handle.idx].idx].data;
    }

    void destroy(Handle handle)
    {
        if (!is_valid(handle))
            return;

        size_t obj_idx = mapping[handle.idx].idx;
        Object<T>& last_obj = objects.back();

        // Point last object's mapping idx to point at the object that will be destroyed
        mapping[last_obj.mapping_idx].idx = obj_idx;

        // Swap the last object with the object to be destroyed, then pop the vec
        objects[obj_idx] = last_obj;
        objects.pop_back();

        // Set the mapping to be inactive for the object that was destroyed
        mapping[handle.idx].active = false;
    }

    inline bool is_valid(Handle handle)
    {
        size_t idx = handle.idx;
        uint64_t gen = handle.generation;
        return idx < mapping.size() && mapping[idx].active&& gen == mapping[idx].generation;
    }
};

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
}*/

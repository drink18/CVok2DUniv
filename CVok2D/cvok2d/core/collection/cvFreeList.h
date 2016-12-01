#pragma once

#include <vector>

// a list built on top of vector so elements are in continuous memory most of time
// while you still can do alloc/free efficiently without copying everything

template<typename T, typename Handle>
class cvFreeList
{
private:
	struct FREESLOT
	{
		int m_nextFreeSlot;
	};
	enum {INVALID_INDEX = -1};
public:
	cvFreeList(int capacity = 512);

public:
	Handle alloc(); // allcoate from a know free slot
	void free(Handle handle);
	T& getAt(Handle handle) { return m_vector[handle.getVal()]; }
private:
	std::vector<T> m_vector;
	int m_nextFreeSlot;
};

template<typename T, typename Handle>
cvFreeList<T, Handle>::cvFreeList(int capacity)
{
	static_assert(sizeof(T) >= sizeof(FREESLOT), "T must be bigger than FREESLOT");
	m_vector.reserve(capacity);
	m_vector.resize(capacity);
	for (int i = 0; i < capacity; ++i)
	{
		FREESLOT* fs = reinterpret_cast<FREESLOT*>(&m_vector[i]);
		fs->m_nextFreeSlot = i < capacity -1 ?  i + 1 : INVALID_INDEX;
	}

	m_nextFreeSlot = 0;
}

template<typename T, typename Handle>
Handle cvFreeList<T, Handle>::alloc()
{
	Handle h(m_nextFreeSlot);
	FREESLOT* fs = reinterpret_cast<FREESLOT*>(&m_vector[m_nextFreeSlot]);
	m_nextFreeSlot = fs->m_nextFreeSlot;
	return h;
}

template<typename T, typename Handle>
void cvFreeList<T, Handle>::free(Handle handle)
{
	FREESLOT* fs = reinterpret_cast<FREESLOT*>(&m_vector[handle.getVal()]);
	fs->m_nextFreeSlot = m_nextFreeSlot;
	m_nextFreeSlot = handle.getVal();
}




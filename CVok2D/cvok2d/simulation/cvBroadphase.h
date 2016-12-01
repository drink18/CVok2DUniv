#pragma once

#include <cstdint>
#include <vector>
#include <map>
#include <unordered_map>

#include <core/cvHandle.h>
#include <core/collection/cvFreeList.h>
#include <core/cvAabb.h>

class cvAabb;
class cvBody;

typedef  cvHandle<int32_t, (int32_t)0x7fffffff> cvBroadphaseHandle;

struct cvBroadphaseCInfo
{
public:
	cvBroadphaseCInfo();
};

class cvBroadphase
{
public:
	template<typename T>
	struct NodeEndPoint_T
	{
		T m_Val;
		int32_t m_NodeHandleAndMinFlag;

		enum { BPHANDLE_MASK = 0x7FFFFFFF };

		NodeEndPoint_T()
			: m_NodeHandleAndMinFlag(cvBroadphaseHandle::invalid())
		{}
		cvBroadphaseHandle getBPHandle() const { return  BPHANDLE_MASK & m_NodeHandleAndMinFlag; }
		void setBPHandle(cvBroadphaseHandle handle) 
		{ 
			m_NodeHandleAndMinFlag = handle.getVal() | (~BPHANDLE_MASK & m_NodeHandleAndMinFlag);
		}
		bool getIsMin() const { return (~BPHANDLE_MASK & m_NodeHandleAndMinFlag) != 0; }
		bool getIsMax() const{ return !getIsMin(); }
		void setIsMin(bool isMin) { isMin ? m_NodeHandleAndMinFlag |= ~BPHANDLE_MASK : m_NodeHandleAndMinFlag &= BPHANDLE_MASK; }
		T getVal() const { return m_Val; }
	};

	typedef NodeEndPoint_T<float> NodeEndPoint;

	struct BPPair
	{
		cvBroadphaseHandle m_h1;
		cvBroadphaseHandle m_h2;
		
		int64_t getSortKey() const {
			int64_t key;
			key = m_h1.getVal() > m_h2.getVal() ? 
				(int64_t)m_h2.getVal() << 32 | m_h1.getVal() 
				: (int64_t)m_h1.getVal() << 32 | m_h2.getVal();
			return key;
		}
	public:
		BPPair() : m_h1(cvBroadphaseHandle::invalid()), m_h2(cvBroadphaseHandle::invalid()) {}
		BPPair(cvBroadphaseHandle h1, cvBroadphaseHandle h2)
			:m_h1(h1), m_h2(h2)
		{
		}

		bool operator==(const BPPair& other) const
		{
			return getSortKey() == other.getSortKey();
		}

		struct compBPPair
		{
			size_t operator()(const BPPair& p1) const
			{
				return p1.getSortKey();
			}
		};
	};

public:
	cvBroadphase(const cvBroadphaseCInfo& cinfo);

public:
	virtual void updateOneNode(cvBroadphaseHandle handle, const cvAabb& newAabb) = 0;
	virtual cvBroadphaseHandle addNode(const cvAabb& nodeAabb) = 0;
	virtual void removeNode(cvBroadphaseHandle handle) = 0;
	virtual void getAllPairs(std::vector<BPPair>& pairs) = 0;

protected:

	virtual bool addPair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) = 0;
	virtual bool removePair(const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2) = 0;
};

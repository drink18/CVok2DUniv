#include "acd.h"

namespace acd
{
	using namespace std;
	bool Loop::AreNeighbour(size_t idx0, size_t idx1) const
	{
		const size_t totalV = ptCount();
		int diff = abs((int)idx0 - (int)idx1);
		return (diff == 1) || (diff == totalV - 1);
	}

	void HullLoop::insertAfterIdx(PolyVertIdx after, PolyVertIdx idx)
	{
		auto iter = find(ptIndicies.begin(), ptIndicies.end(), after);
		cvAssert(iter != ptIndicies.end());
		ptIndicies.insert(iter + 1, idx);
	}
}
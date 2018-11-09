#include "acd.h"

namespace acd
{
	using namespace std;
	bool Loop::AreNeighbour(int idx0, int idx1) const
	{
		const size_t totalV = Vertices.size();
		int diff = abs(idx0 - idx1);
		return (diff == 1) || (diff == totalV - 1);
	}

	void HullLoop::insertAfterIdx(int after, int idx)
	{
		auto iter = find(ptIndicies.begin(), ptIndicies.end(), after);
		cvAssert(iter != ptIndicies.end());
		ptIndicies.insert(iter + 1, idx);
	}
}
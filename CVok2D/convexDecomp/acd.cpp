#include "acd.h"
#include <algorithm>

namespace acd
{
	static float Vec2Cross(cvVec2f v1, cvVec2f v2)
	{
		return v1.x * v2.y - v1.y * v2.x;
	}

	static float DistToLine(cvVec2f p0, cvVec2f p1, cvVec2f p)
	{
		auto n = (p1 - p0).getNormalized();
		return Vec2Cross(n, p - p0);
	}

	static bool PtInTriangle(cvVec2f p0, cvVec2f p1, cvVec2f p2, cvVec2f p)
	{
		float p0p1p = Vec2Cross(p - p0, p1 - p0);
		float pp1p2 = Vec2Cross(p2 - p, p1 - p);
		float p0pp2 = Vec2Cross(p2 - p0, p - p0);

		return (p0p1p >= 0 && pp1p2 >= 0 && p0pp2 >= 0)
			|| (p0p1p <= 0 && pp1p2 <= 0 && p0pp2 <= 0);
	}

	void _quickHull(const vector<cvVec2f>& origPoints, int idxP0, int idxP1, vector<int>& available, vector<int>& hullLoop);
	vector<int> _quickHull(const Loop& loop)
	{
		auto hullLoop = vector<int>();

		auto minX = 1e10f;
		auto maxX = -1e10f;
		auto minIdx = 0;
		auto maxIdx = loop.Vertices.size() - 1;
		//find idx of min, max x
		for (int i = 0; i < loop.Vertices.size(); ++i)
		{
			auto pt = loop.Vertices[i];
			if (pt.x < minX)
			{
				minIdx = i;
				minX = pt.x;
			}
			if (pt.x > maxX)
			{
				maxIdx = i;
				maxX = pt.x;
			}
		}

		auto p0 = loop.Vertices[minIdx];
		auto p1 = loop.Vertices[maxIdx];


		auto upper = vector<int>();
		auto lower = vector<int>();

		for (int i = 0; i < loop.Vertices.size(); ++i)
		{
			if (i == minIdx || i == maxIdx)
				continue;

			auto pt = loop.Vertices[i];
			auto d = DistToLine(p0, p1, pt);
			if (d >= 0)
				upper.push_back(i);
			else
				lower.push_back(i);

		}
		hullLoop.push_back(minIdx);
		hullLoop.push_back(maxIdx);

		_quickHull(loop.Vertices, minIdx, maxIdx, upper, hullLoop);
		_quickHull(loop.Vertices, maxIdx, minIdx, lower, hullLoop);

		sort(hullLoop.begin(), hullLoop.end());
		return hullLoop;
	}

	void _quickHull(const vector<cvVec2f>& origPoints, int idxP0, int idxP1, vector<int>& available, vector<int>& hullLoop)
	{
		if (available.size() == 0)
			return;

		//find pts further away from line
		cvVec2f p0 = origPoints[idxP0];
		cvVec2f p1 = origPoints[idxP1];

		auto furtherest = 0.0f;

		auto bestIdx = -1;
		for (int i = 0; i < available.size(); ++i)
		{
			auto p = origPoints[available[i]];
			auto d = DistToLine(p0, p1, p);
			if (d > furtherest)
			{
				furtherest = d;
				bestIdx = available[i];
			}
		}

		// no points on correct side
		if (bestIdx == -1)
			return;


		//hullLoop.Add(bestIdx);
		hullLoop.insert(find(hullLoop.begin(), hullLoop.end(), idxP1) + 1, bestIdx);
		available.erase(find(available.begin(), available.end(), bestIdx));

		vector<int> removedPts;
		for (int i = 0; i < available.size(); ++i)
		{
			auto p = origPoints[available[i]];
			if (PtInTriangle(p0, origPoints[bestIdx], p1, p))
				removedPts.push_back(available[i]);
		}

		for (int i = 0; i < removedPts.size(); ++i)
			available.erase(find(available.begin(), available.end(), removedPts[i]));

		_quickHull(origPoints, idxP0, bestIdx, available, hullLoop);
		_quickHull(origPoints, bestIdx, idxP1, available, hullLoop);
	}

	vector<Bridge> _findAllPockets(const vector<int>& hull, const Loop& loop)
	{
		vector<Bridge> bridge;
		const int DEC = 0;
		const int ACC = 1;
		int order = hull[1] - hull[0] > 0 ? ACC : DEC;

		int prevIdx = hull[0];
		for (int i = 1; i <= hull.size(); ++i)
		{
			int curIdx = hull[i % hull.size()];
			if (!loop.AreNeighbour(prevIdx, curIdx))
			{
				Bridge b;
				b.idx0 = prevIdx;
				b.idx1 = curIdx;
				if (prevIdx > curIdx)
				{
					int i0 = loop.nextIdx(prevIdx);
					int i1 = loop.prevIdx(curIdx);
					for (int j = i0; j <=  i1; j++)
					{
						b.notches.push_back(j);
					}
				}
				else
				{ 
					for (int j = prevIdx + 1; j < curIdx; j++)
					{
						b.notches.push_back(j);
					}
				}
				cvAssert(b.notches.size() > 0);
				bridge.push_back(b);
			}
			prevIdx = curIdx;

		}

		return bridge;
	}
	
	WitnessPt _pickCW(const Loop& loop, const vector<int>& hull, const vector<Bridge>& pockets)
	{
		WitnessPt cw;

		float bestScore = 1e20f;
		int bestPocketIdx = -1;
		int bestPtIdx = -1;

		for (int ip= 0; ip < pockets.size(); ++ip)
		{
			auto& p = pockets[ip];
			auto& b0 = loop.Vertices[p.idx0];
			auto& b1 = loop.Vertices[p.idx1];
			for (int i = 0; i < p.notches.size(); ++i)
			{
				auto& ni = p.notches[i];
				float d = abs(DistToLine(b0, b1, loop.Vertices[ni]));
				if (d < bestScore)
				{
					bestScore = d;
					bestPtIdx = ni;
				}
			}
		}

		cw.Concavity = bestScore;
		cw.ptIndex = bestPtIdx;

		return cw;

	}
}
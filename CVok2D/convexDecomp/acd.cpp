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

	void _quickHull(const Loop& loop, PolyVertIdx idxP0, PolyVertIdx idxP1, 
		vector<PolyVertIdx>& available, HullLoop& hullLoop);

	HullLoop _quickHull(const Loop& loop)
	{
		HullLoop hullLoop;

		auto minX = 1e10f;
		auto maxX = -1e10f;
		PolyVertIdx minIdx(0);
		PolyVertIdx maxIdx(loop.ptCount() - 1);
		//find idx of min, max x
		for (PolyVertIdx i = PolyVertIdx(0); i < PolyVertIdx(loop.ptCount()); ++i)
		{
			auto pt = loop[i];
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

		auto p0 = loop[minIdx];
		auto p1 = loop[maxIdx];


		auto upper = vector<PolyVertIdx>();
		auto lower = vector<PolyVertIdx>();

		for (PolyVertIdx i = PolyVertIdx(0); i < PolyVertIdx(loop.ptCount()); ++i)
		{
			if (i == minIdx || i == maxIdx)
				continue;

			auto pt = loop[i];
			auto d = DistToLine(p0, p1, pt);
			if (d >= 0)
				upper.push_back(i);
			else
				lower.push_back(i);

		}

		hullLoop.addIndex(minIdx);
		hullLoop.addIndex(maxIdx);

		_quickHull(loop,  minIdx, maxIdx, upper, hullLoop);
		_quickHull(loop,  maxIdx, minIdx, lower, hullLoop);

		hullLoop.sort();
		return hullLoop;
	}

	void _quickHull(const Loop& loop, PolyVertIdx idxP0, PolyVertIdx idxP1, 
		vector<PolyVertIdx>& available, HullLoop& hullLoop)
	{
		if (available.size() == 0)
			return;

		//find pts further away from line
		cvVec2f p0 = loop[idxP0];
		cvVec2f p1 = loop[idxP1];

		auto furtherest = 0.0f;

		PolyVertIdx bestIdx(-1);
		for (int i = 0; i < available.size(); ++i)
		{
			auto p = loop[available[i]];
			auto d = DistToLine(p0, p1, p);
			if (d > furtherest)
			{
				furtherest = d;
				bestIdx = available[i];
			}
		}

		// no points on correct side
		if (bestIdx == PolyVertIdx(-1))
			return;


		hullLoop.insertAfterIdx(idxP1, bestIdx);
		available.erase(find(available.begin(), available.end(), bestIdx));

		vector<PolyVertIdx> removedPts;
		for (int i = 0; i < available.size(); ++i)
		{
			auto p = loop[available[i]];
			if (PtInTriangle(p0, loop[bestIdx], p1, p))
				removedPts.push_back(available[i]);
		}

		for (int i = 0; i < removedPts.size(); ++i)
			available.erase(find(available.begin(), available.end(), removedPts[i]));

		_quickHull(loop, idxP0, bestIdx, available, hullLoop);
		_quickHull(loop, bestIdx, idxP1, available, hullLoop);
	}

	vector<Bridge> _findAllPockets(const HullLoop& hull, const Loop& loop)
	{
		vector<Bridge> bridge;

		HullIdx prevHullIdx = HullIdx(0);
		PolyVertIdx prevIdx = hull[prevHullIdx];
		for (int i = 1; i <= hull.pointCount(); ++i)
		{
			HullIdx curHullIdx = HullIdx(i % hull.pointCount());
			PolyVertIdx curIdx = hull[curHullIdx];
			if (!loop.AreNeighbour(prevIdx, curIdx))
			{
				Bridge b;
				b.idx0 = prevHullIdx;
				b.idx1 = curHullIdx;
				if (prevIdx > curIdx)
				{
					PolyVertIdx i0 = loop.nextIdx(prevIdx);
					PolyVertIdx i1 = loop.prevIdx(curIdx);
					for (auto j = i0; j <= i1; j++)
					{
						b.notches.push_back(PolyVertIdx(j));
					}
				}
				else
				{ 
					for (auto j = prevIdx + 1; j < curIdx; j++)
					{
						b.notches.push_back(j);
					}
				}
				cvAssert(b.notches.size() > 0);
				bridge.push_back(b);
			}
			prevIdx = curIdx;
			prevHullIdx = curHullIdx;
		}

		return bridge;
	}
	
	WitnessPt _pickCW(const Loop& loop, const HullLoop& h, const vector<Bridge>& pockets)
	{
		WitnessPt cw;

		float bestScore = -1e20f;
		int bestPocketIdx = -1;
		PolyVertIdx bestPtIdx(-1);

		for (int ip= 0; ip < pockets.size(); ++ip)
		{
			auto& p = pockets[ip];
			auto& b0 = loop[h[p.idx0]];
			auto& b1 = loop[h[p.idx1]];
			for (int i = 0; i < p.notches.size(); ++i)
			{
				auto& ni = p.notches[i];
				float d = abs(DistToLine(b0, b1, loop[ni]));
				if (d > bestScore)
				{
					bestScore = d;
					bestPtIdx = ni;
					bestPocketIdx = ip;
				}
			}
		}

		cw.Concavity = bestScore;
		cw.ptIndex = bestPtIdx;
		cw.loopIndex = 0;
		cw.pocketIdx = bestPocketIdx;

		return cw;
	}

	bool isValidCutPt(const Loop& loop, const HullLoop& h, const vector<Bridge>& pockets,
		int curPocketIdx, PolyVertIdx cwpIdx, HullIdx hullPtIdx)
	{
		auto& curPocket = pockets[curPocketIdx];
		cvVec2f cwPt = loop[cwpIdx];

		for (auto& p : pockets)
		{
			// ignore bridge vertices
			// TODO: actually, bridge point of current pocket can be a candidate too
			if (hullPtIdx == p.idx0 || hullPtIdx == p.idx1)
				return false;
		}

		//float dl = DistToLine(cwPt, vts[loop.prevIdx(cwpIdx)], curHullVtx);
		//float dr = DistToLine(vts[loop.nextIdx(cwpIdx)], cwPt, curHullVtx);

		//if (dl < 0 || dr < 0)
		//	return false;

		return true;
	}
	PolyVertIdx _findBestCutPt(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockets,
		const WitnessPt& cwp)
	{
		auto& pocket = pockets[cwp.pocketIdx];

		// find a point on hull that forms a line with witness point, that does not intersect with
		// other pockets


		for(int ih = 0; ih < hull.pointCount(); ++ih)
		{
			if (isValidCutPt(loop, hull, pockets, cwp.pocketIdx, cwp.ptIndex, HullIdx(ih)))
			{
				// return index on original polygon
				return hull[HullIdx(ih)];
			}
		}

		cvAssert(false);
		return PolyVertIdx(0);
	}

	vector<Polygon> _resolveLoop(const Loop& loop, const HullLoop& hull, const WitnessPt& cwp
		, int cutPtIdx)
	{
		vector<Polygon> poly;
		return poly;
	}
}
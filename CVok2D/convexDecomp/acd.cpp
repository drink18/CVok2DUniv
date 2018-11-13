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

	Loop::Loop(const vector<cvVec2f>& vtx) 
		:_vertices(vtx)
	{
		if (vtx.size() > 2)
		{
			cvVec2f v0 = _vertices[0];
			cvVec2f v1 = _vertices[1];
			cvVec2f v2 = _vertices[2];
			if (Vec2Cross(v1 - v0, v2 - v1) > 0)
				reverse(_vertices.begin(), _vertices.end());
		}
		updateNormals();
	}

	void Loop::updateNormals()
	{
		_normals.clear();
		_normals.reserve(_vertices.size());

		if (ptCount() < 2)
			return;

		for (PolyVertIdx i = beginIdx(); i <= endIdx(); ++i)
		{
			cvVec2f v = (*this)[i];
			cvVec2f nv = (*this)[nextIdx(i)];
			cvVec2f e = nv - v;
			cvVec2f n(-e.y, e.x);
			n.normalize();
			_normals.push_back(n);
		}
	}

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

		//hullLoop.sort();
		cvVec2f v0 = loop[hullLoop[HullIdx(0)]];
		cvVec2f v1 = loop[hullLoop[HullIdx(1)]];
		cvVec2f v2 = loop[hullLoop[HullIdx(2)]];
		float a = Vec2Cross(v1 - v0, v2 - v1);
		if (a > 0)
		{
			reverse(hullLoop.begin(), hullLoop.end());
		}
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
				//TODO : THIS IS BUGGY
				Bridge b;
				b.idx0 = prevIdx;
				b.idx1 = curIdx;
				for (PolyVertIdx idx = loop.nextIdx(b.idx0); idx != b.idx1; idx = loop.nextIdx(idx))
				{
					b.notches.push_back(idx);
				}

				//sort(b.notches.begin(), b.notches.end());
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
			auto& b0 = loop[p.idx0];
			auto& b1 = loop[p.idx1];
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

	bool isValidCutPt(const Loop& loop,  PolyVertIdx cwpIdx, PolyVertIdx vIdx)
	{

		if(vIdx != cwpIdx)
		{
			cvVec2f v = loop[vIdx];
			cvVec2f cwPt = loop[cwpIdx];
			cvVec2f prev = loop[loop.prevIdx(cwpIdx)];
			cvVec2f next = loop[loop.nextIdx(cwpIdx)];

			cvVec2f v1 = cwPt - prev;
			cvVec2f v2 = cwPt - next;
			cvVec2f vec = v - cwPt;
	
			float a = Vec2Cross(vec, v2);
			float b = Vec2Cross(vec, v1);
			float c = Vec2Cross(v2, v1);

			return (a * c < 0) && (b * c > 0);
		}
		return false;
	}

	CutLine _findCutLine(const Loop& loop, PolyVertIdx origin)
	{

		PolyVertIdx cutVtx(-1);
		cvVec2f lineDir;
		for (auto idx = loop.beginIdx(); idx <= loop.endIdx(); ++idx)
		{
			if (isValidCutPt(loop, origin, idx))
			{
				cutVtx = idx;
				lineDir = (loop[idx] - loop[origin]).getNormalized();
				break;
			}
		}

		if (cutVtx.val() == -1)
		{
			// can't find a vertex as cut point , use normal as cut direction and find 
			// best cutting point
			lineDir = -(loop.normal(origin) + loop.normal(loop.prevIdx(origin)));
			lineDir.normalize();
		}

		CutLine line;
		line.lineDir = lineDir;
		line.orgin = origin;

		return line;
	}

	PolyVertIdx _findBestCutPt(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockets,
		const WitnessPt& cwp)
	{
		auto& pocket = pockets[cwp.pocketIdx];

		CutLine cutLine = _findCutLine(loop, cwp.ptIndex);

		// can't find a vertex as cut point , use normal as cut direction and find 
		// best cutting point
		cvVec2f lineDir = cutLine.lineDir;
		lineDir.normalize();
		cvVec2f lineNorm(-lineDir.y, lineDir.x);

		vector<cvVec2f> intersects;
		vector<float> dists;
		vector<PolyVertIdx> vtxIndices;
		
		cvVec2f cwPt = loop[cwp.ptIndex];
		const float eps = 1e-5f;

		for (auto idx = loop.beginIdx(); idx <= loop.endIdx(); ++idx)
		{
			PolyVertIdx ni = loop.nextIdx(idx);
			if (idx == cwp.ptIndex) continue;
			if (idx == loop.prevIdx(cwp.ptIndex)) continue;
			if (idx == loop.nextIdx(cwp.ptIndex)) continue;

			//if (up != nup)
			{
				cvVec2f v = loop[idx];
				cvVec2f nv = loop[ni];
				// intersecting, compute distance etc
				float t = (v - cwPt).dot(lineNorm) / (v - nv).dot(lineNorm);
				cvVec2f intersect = v * (1 - t) + nv * t;
				float dist = (intersect - cwPt).dot(lineDir); 
				if (t < -eps || t > eps + 1.0f)
					dist = -1e10f;

				if (dist > -eps)
				{
					vtxIndices.push_back(idx);
					intersects.push_back(intersect);
					dists.push_back(dist);
				}
			}
		}

		//pick closet 
		float bestDist = 1e10f;
		PolyVertIdx bestPtIdx(-1);
		for(int i = 0; i < vtxIndices.size(); ++i)
		{
			float d = dists[i];
			if (d < bestDist && d > - CV_FLOAT_EPS)
			{
				bestDist = d;
				bestPtIdx = vtxIndices[i];
			}
		}

		cvAssert(bestPtIdx.val() != -1);

		return bestPtIdx;
	}

	vector<Loop> _resolveLoop(const Loop& loop)
	{
		vector<Loop> poly;
		
		Loop polyLoop(loop);
		auto hullLoop = _quickHull(polyLoop);

		// already convex
		if (hullLoop.pointCount() == polyLoop.ptCount())
		{
			poly.push_back(loop);
			return poly;
		}

		auto bridges = _findAllPockets(hullLoop, polyLoop);
		cvAssert(bridges.size() > 0);

		// pick deepest notches 
		auto cw = _pickCW(polyLoop, hullLoop, bridges);

		// find best cut point
		PolyVertIdx cutPointIdx = _findBestCutPt(polyLoop, hullLoop, bridges, cw);

		// split loop
		Loop loop1;
		for (PolyVertIdx i = PolyVertIdx(cw.ptIndex); ; i = loop.nextIdx(i))
		{
			loop1.AddVertex(loop[i]);
			if (i == PolyVertIdx(cutPointIdx))
				break;
		}
		poly.push_back(loop1);

		Loop loop2;
		for (PolyVertIdx i = PolyVertIdx(cutPointIdx); ; i = loop.nextIdx(i))
		{
			loop2.AddVertex(loop[i]);
			if (i == cw.ptIndex)
				break;
		}
		poly.push_back(loop2);
		

		return poly;
	}
}
#include "acd.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <string>

namespace acd
{
	using namespace std; 

	static float Vec2Cross(cvVec2f v1, cvVec2f v2)
	{
		return v1.x * v2.y - v1.y * v2.x;
	}

	static float DistToLine(cvVec2f p0, cvVec2f p1, cvVec2f p)
	{
		auto n = (p1 - p0).getNormalized();
		return Vec2Cross(n, p - p0);
	}

	static cvVec2f ClosestPtOnSeg(cvVec2f p, cvVec2f p0, cvVec2f p1)
	{
		cvVec2f d = p1 - p0;
		float len = d.length();
		d /= len;
		float t = (p - p0).dot(d) / len;
		t = max(min(t, 1.0f), 0.0f);
		return p0 * (1 - t) + p1 * t;
	}

	static bool PtInTriangle(cvVec2f p0, cvVec2f p1, cvVec2f p2, cvVec2f p)
	{
		float p0p1p = Vec2Cross(p - p0, p1 - p0);
		float pp1p2 = Vec2Cross(p2 - p, p1 - p);
		float p0pp2 = Vec2Cross(p2 - p0, p - p0);

		return (p0p1p >= 0 && pp1p2 >= 0 && p0pp2 >= 0)
			|| (p0p1p <= 0 && pp1p2 <= 0 && p0pp2 <= 0);
	}

	static bool IntersectRayLine(cvVec2f o, cvVec2f dir, cvVec2f from, cvVec2f to, float& outT, cvVec2f& intersect)
	{
		cvVec2f n(-dir.y, dir.x);
		outT = (o - from).dot(n) / (to - from).dot(n);
		
		intersect = from * (1 - outT) + to * outT;
		return outT >= 0 && outT <= 1.0f;
	}

	static bool IntersectLines(cvVec2f a0, cvVec2f a1, cvVec2f b0, cvVec2f b1, float& outT, cvVec2f& outPt)
	{
		cvVec2f da = (a1 - a0).getNormalized();
		return IntersectRayLine(a0, da, b0, b1, outT, outPt);
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

		_quickHull(loop, minIdx, maxIdx, upper, hullLoop);
		_quickHull(loop, maxIdx, minIdx, lower, hullLoop);

		vector<cvVec2f> hullVts;
		hullVts.reserve(loop.ptCount());
		for (PolyVertIdx idx : hullLoop)
			hullVts.push_back(loop[idx]);

		if (_testWinding(hullVts) == Winding::CCW)
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

	HullLoop quickHull(const Polygon& polygon)
	{
		const Loop& loop = polygon.outterLoop();
		return _quickHull(loop);
	}

	vector<Pocket> findAllPockets(const HullLoop& hull, const Loop& loop)
	{
		vector<Pocket> bridge;

		HullIdx prevHullIdx = HullIdx(0);
		PolyVertIdx prevIdx = hull[prevHullIdx];
		for (int i = 1; i <= hull.pointCount(); ++i)
		{
			HullIdx curHullIdx = HullIdx(i % hull.pointCount());
			PolyVertIdx curIdx = hull[curHullIdx];
			if (!loop.AreNeighbour(prevIdx, curIdx))
			{
				//TODO : THIS IS BUGGY
				Pocket b;
				b.idx0 = prevIdx;
				b.idx1 = curIdx;
				for (PolyVertIdx idx = loop.nextIdx(b.idx0); idx != b.idx1; idx = loop.nextIdx(idx))
				{
					b.notches.push_back(idx);
				}

				cvAssert(b.notches.size() > 0);
				bridge.push_back(b);
			}
			prevIdx = curIdx;
			prevHullIdx = curHullIdx;
		}

		return bridge;
	}

	WitnessPt _pickCW_inner(const Polygon& poly)
	{
		WitnessPt cw;
		cvAssert(poly.hasHole());

		auto& loops = poly.loops;
		// pick first hole.....
		const Loop& loop = poly.loops[1];
		const Loop& outLoop = poly.outterLoop();

		cvVec2f bestPt;
		PolyVertIdx bestIdx(-1);
		float bestD = 1e10f;

		for (PolyVertIdx inIdx(0); inIdx < PolyVertIdx(loop.ptCount()); ++inIdx)
		{
			cvVec2f inV = loop[inIdx];
			for (PolyVertIdx i = PolyVertIdx(0); i < PolyVertIdx(outLoop.ptCount()); ++i)
			{
				cvVec2f v = outLoop[i];
				cvVec2f nv = outLoop[outLoop.nextIdx(i)];

				cvVec2f cp = ClosestPtOnSeg(inV, v, nv);
				float d = (cp - inV).sqrLength();
				if (d < bestD)
				{
					bestD = d;
					bestIdx = inIdx;
					bestPt = cp;
				}
			}
		}

		cw.Concavity = 1000;
		cw.loopIndex = 1;
		cw.pocketIdx = -1;
		cw.ptIndex = bestIdx;

		return cw;
	}

	float _computeConcavity_out(const Loop& loop, const Pocket& pocket, int notchIdx)
	{
			cvVec2f b0 = loop[pocket.idx0];
			cvVec2f b1 = loop[pocket.idx1];

			PolyVertIdx ni = pocket.notches[notchIdx];
			float d = abs(DistToLine(b0, b1, loop[ni]));
			return d;
	}

	float _computeConcavity_in(const Loop& loop, const Loop& outLoop)
	{
		return 100;
	}

	WitnessPt _pickCW_outter(const Loop& loop,  const vector<Pocket>& pockets)
	{
		WitnessPt cw;

		float bestScore = -1e20f;
		int bestPocketIdx = -1;
		PolyVertIdx bestPtIdx(-1);

		for (int ip = 0; ip < pockets.size(); ++ip)
		{
			auto& p = pockets[ip];
			auto& b0 = loop[p.idx0];
			auto& b1 = loop[p.idx1];
			for (int i = 0; i < p.notches.size(); ++i)
			{
				float d = _computeConcavity_out(loop, p, i);
				if (d > bestScore)
				{
					bestScore = d;
					bestPtIdx = p.notches[i];
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

	WitnessPt pickCW(const Polygon& poly, const HullLoop& h, const vector<Pocket>& pockets)
	{
		if (poly.hasHole())
		{
			return _pickCW_inner(poly);
		}
		else
		{
			return _pickCW_outter(poly.outterLoop(), pockets);
		}
	}

	bool isValidCutPt(const Polygon& poly, const WitnessPt& wp, PolyVertIdx vIdx)
	{
		const Loop& outterLoop = poly.outterLoop();
		const Loop& innerLoop = poly.loops[wp.loopIndex];
		
		PolyVertIdx cwpIdx = wp.ptIndex;
		if (vIdx != cwpIdx)
		{
			cvVec2f v = outterLoop[vIdx];
			cvVec2f cwPt = innerLoop[cwpIdx];
			cvVec2f prev = innerLoop[innerLoop.prevIdx(cwpIdx)];
			cvVec2f next = innerLoop[innerLoop.nextIdx(cwpIdx)];

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

	CutLine _findCutLine_outter(const Polygon& polygon, const WitnessPt& wp)
	{
		PolyVertIdx cutVtx(-1);
		cvVec2f lineDir;
		const Loop& loop = polygon.outterLoop();
		const Loop& origLoop = polygon.loops[wp.loopIndex];
		cvVec2f originPt = origLoop[wp.ptIndex];
		cvVec2f originNormal = origLoop.normal(wp.ptIndex);
		cvVec2f prevOriginNormal = origLoop.normal(origLoop.prevIdx(wp.ptIndex));
		
		for (auto idx = loop.beginIdx(); idx <= loop.endIdx(); ++idx)
		{
			if (isValidCutPt(polygon, wp, idx))
			{ cutVtx = idx;
				lineDir = (loop[idx] - originPt).getNormalized();
				break;
			}
		}

		if (cutVtx.val() == -1)
		{
			// can't find a vertex as cut point , use blended normal as cut direction and find 
			// best cutting point
			lineDir = -(originNormal + prevOriginNormal);
			lineDir.normalize();
		}

		CutLine line;
		line.lineDir = lineDir;
		line.orgin = wp;
		line.originPt = originPt;

		return line;
	}

	CutLine _findCutLine_inner(const Polygon& poly, const WitnessPt& wp)
	{
		PolyVertIdx cutVtx(-1);
		cvVec2f lineDir;

		const Loop& loop = poly.outterLoop();
		const Loop& innerLoop = poly.loops[wp.loopIndex];
		cvVec2f orignPt = innerLoop[wp.ptIndex];
		cvVec2f orignNorm = innerLoop.normal(wp.ptIndex);
		cvVec2f prevOrignNorm = innerLoop.normal(innerLoop.prevIdx(wp.ptIndex));

		// if there is a point that could directly resolve witness point, use it to construct direction
		for (auto idx = loop.beginIdx(); idx <= loop.endIdx(); ++idx)
		{
			if (isValidCutPt(poly, wp, idx))
			{
				cutVtx = idx;
				lineDir = (loop[idx] - orignPt).getNormalized();
				break;
			}
		}

		if (cutVtx.val() == -1)
		{
			// can't find a vertex as cut point , use blended normal as cut direction and find 
			// best cutting point
			lineDir = orignNorm + prevOrignNorm;
			lineDir.normalize();
		}

		CutLine line;
		line.lineDir = lineDir;
		line.originPt = orignPt;
		line.orgin = wp;

		return line;
	}

	CutLine findCutLine(const Polygon& polygon, const WitnessPt& wp)
	{
		if (polygon.hasHole())
		{
			return _findCutLine_inner(polygon, wp);
		}
		else
		{
			return _findCutLine_outter(polygon, wp);
		}
	}

	PolyVertIdx findBestCutPt(const Polygon& poly, const HullLoop& hull, const vector<Pocket>& pockets,
		const WitnessPt& cwp)
	{
		const Loop& outLoop = poly.outterLoop();

		CutLine cutLine = findCutLine(poly, cwp);

		// can't find a vertex as cut point , use normal as cut direction and find 
		// best cutting point
		cvVec2f lineDir = cutLine.lineDir;
		lineDir.normalize();
		cvVec2f lineNorm(-lineDir.y, lineDir.x);

		vector<cvVec2f> intersects;
		vector<float> dists;
		vector<PolyVertIdx> vtxIndices;
		
		cvVec2f cwPt = poly.loops[cwp.loopIndex][cwp.ptIndex];
		const float eps = 1e-5f;

		for (auto idx = outLoop.beginIdx(); idx <= outLoop.endIdx(); ++idx)
		{
			PolyVertIdx ni = outLoop.nextIdx(idx);
			if (cwp.loopIndex == 0)
			{
				if (idx == cwp.ptIndex) continue;
				if (idx == outLoop.prevIdx(cwp.ptIndex)) continue;
				if (idx == outLoop.nextIdx(cwp.ptIndex)) continue;
			}

			cvVec2f v = outLoop[idx];
			cvVec2f nv = outLoop[ni];

			// intersecting, compute distance etc
			float t;
			cvVec2f intersect;

			IntersectRayLine(cwPt, lineDir, v, nv, t, intersect);
			float dist = (intersect - cwPt).dot(lineDir);
			//  t > 1.0f -eps makes sure that we only pick start point
			if (t < -eps || t > 1.0f - eps)
				dist = -1e10f;

			if (dist > -eps)
			{
				vtxIndices.push_back(idx);
				intersects.push_back(intersect);
				dists.push_back(dist);
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

		static int a = 0;
		if (bestPtIdx.val() == -1)
			++a;
		cvAssert(bestPtIdx.val() != -1);

		return bestPtIdx;
	}

	vector<Polygon> _resolveLoop_OneStep(const Polygon& polygon)
	{
		vector<Polygon> retPolygons;
		
		auto& loop = polygon.outterLoop();

		// already convex
		if (polygon.isConvex())
		{
			retPolygons.push_back(polygon);
			return retPolygons;
		}

		//cvAssert(bridges.size() > 0);

		// pick deepest notches 
		//auto cw = pickCW(polygon, hullLoop, bridges);
		auto cw = polygon.findWitnessPt();
		

		// find best cut point
		PolyVertIdx cutPointIdx = polygon.findBestCutPt(cw);

		if (cw.loopIndex > 0)
		{
			Polygon retPoly;
			// merge inner loop to outer
			const Loop& innerLoop = polygon.loops[cw.loopIndex];
			for(PolyVertIdx outIdx = PolyVertIdx(0); outIdx < PolyVertIdx(loop.ptCount()); ++outIdx)
			{
				cvVec2f outVtx = loop[outIdx];
				retPoly.outterLoop().AddVertex(outVtx);
				if (outIdx == cutPointIdx)
				{
					// start looping inner loop but backwards
					PolyVertIdx innerIdx = cw.ptIndex;
					do 
					{
						cvVec2f innerVtx = innerLoop[innerIdx];
						retPoly.outterLoop().AddVertex(innerVtx);
						innerIdx = innerLoop.prevIdx(innerIdx); // backward looping

						if (innerIdx == cw.ptIndex)
						{
							// duplicate inner cut point
							retPoly.outterLoop().AddVertex(innerLoop[cw.ptIndex]);
							// duplicate outter cut point
							retPoly.outterLoop().AddVertex(outVtx);
						}
					} while (innerIdx != cw.ptIndex);
				}
			}

			// add remaining holes
			for (int i = 2; i < polygon.loops.size(); ++i)
				retPoly.addLoop(polygon.loops[i]);

			retPoly.initializeAll();
			retPolygons.push_back(retPoly);
		}
		else
		{
			// split loop
			Polygon poly1;
			for (PolyVertIdx i = PolyVertIdx(cw.ptIndex); ; i = loop.nextIdx(i))
			{
				auto& nloop = poly1.outterLoop();
				nloop.AddVertex(loop[i]);
				if (i == PolyVertIdx(cutPointIdx))
				{
					break;
				}
			}

			poly1.initializeAll();
			retPolygons.push_back(poly1);

			Polygon poly2;
			for (PolyVertIdx i = PolyVertIdx(cutPointIdx); ; i = loop.nextIdx(i))
			{
				auto& nloop = poly2.outterLoop();
				nloop.AddVertex(loop[i]);
				if (i == cw.ptIndex)
				{
					break;
				}
			}
			poly2.initializeAll();
			retPolygons.push_back(poly2);
		}

		return retPolygons;
	}

	float _polyArea(const vector<cvVec2f>& verts)
	{

		float sum = 0;
		for (int i = 0; i < verts.size(); ++i)
		{
			int ni = (i + 1) % verts.size();
			cvVec2f v = verts[i];
			cvVec2f nv = verts[ni];
			sum += (nv.x - v.x) * (nv.y + v.y);
		}
		return sum;
	}

	Winding _testWinding(const vector<cvVec2f>& verts)
	{
		//https://stackoverflow.com/questions/1165647/how-to-determine-if-a-list-of-polygon-points-are-in-clockwise-order
		float sum = _polyArea(verts);
		return sum > 0 ? Winding::CW : Winding::CCW;
	}

	void writeLoop(ostream& os, const  Loop &loop)
	{
		os << loop.ptCount() << endl;; // vtx count
		auto& verts = loop.getVertsArray();
		for (auto& p : verts)
			os << p.x << " " << p.y << endl;
	}

	void writePolygonInfo(ostream& os, const Polygon& poly)
	{
		vector<Polygon> polys(1, poly);
		writePolygonListInfo(os, polys);
	}

	void writePolygonListInfo(ostream& os, const vector<Polygon>& polys)
	{
		os << polys.size() << endl; //poly count
		for(int i = 0; i < polys.size(); ++i)
		{
			auto& p = polys[i];
			os << p.loops.size() << endl; //loop count
			for (const auto& loop : p.loops)
			{
				writeLoop(os, loop);
			}
		}
	}

	vector<Polygon> readPolygon(ifstream& is)
	{
		int polyCount;
		vector<Polygon> polyList;

		is >> polyCount; 
		for (int ipoly = 0; ipoly < polyCount; ++ipoly)
		{
			int loopCount = 0;
			is >> loopCount;
			Polygon poly;
			for (int iloop = 0; iloop < loopCount; ++iloop)
			{
				int vtxCount;

				is >> vtxCount;
				Loop l;
				for (int iv = 0; iv < vtxCount; ++iv)
				{
					float x, y;
					is >> x >> y;
					l.AddVertex(cvVec2f(x, y));
				}
				poly.addLoop(l);
			}
			poly.initializeAll();
			polyList.push_back(poly);
		}

		return polyList;
	}

	vector<Polygon> _resolveLoop_All(const Polygon& polygon)
	{
		vector<Polygon> retPolys;
		vector<Polygon> todos;
		todos.push_back(polygon);
		while (!todos.empty())
		{
			vector<Polygon> nextTodos;
			for (int i = 0; i < todos.size(); ++i)
			{
				auto& p = todos[i];
				if (p.isConvex())
				{
					retPolys.push_back(p);
				}
				else
				{
					auto res = _resolveLoop_OneStep(p);
					for (auto& nt : res)
						nextTodos.push_back(nt);
				}
			}
			todos = nextTodos;
		}

		return retPolys;
	}

	vector<Polygon> _resolvePolyList(const vector<Polygon>& polygons)
	{
		vector<Polygon> ret;

		for (auto& p : polygons)
		{
			auto onRes = _resolveLoop_All(p);
			ret.insert(ret.end(), onRes.begin(), onRes.end());
		}

		return ret;

	}
}
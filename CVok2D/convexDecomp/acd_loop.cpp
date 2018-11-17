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

	void Loop::fixWinding()
	{
		if (_vertices.size() > 2)
		{
			if(_testWinding(_vertices) == Winding::CCW)
				reverse(_vertices.begin(), _vertices.end());
		}
	}

	Loop::Loop(const vector<cvVec2f>& vtx) 
		:_vertices(vtx),
		_cwPair(PolyVertIdx(-1), PolyVertIdx(-1))
	{
		fixWinding();
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

	void Loop::computePockets(const HullLoop& hull)
	{
		_pockets = findAllPockets(hull, *this);
	}

	void Loop::computeConcavity_out(const HullLoop& hull)
	{
		_concativty.clear();
		_concativty.resize(ptCount(), 0);
		for (Pocket& p : _pockets)
		{
			for (int ni = 0; ni < p.notches.size(); ++ni)
			{
				float c = _computeConcavity_out(*this, p, ni);
				_concativty[p.notches[ni].val()] = c;
			}
		}
	}

	void Loop::computeConcavity_in()
	{
		_concativty.clear();
		_concativty.resize(ptCount(), 100);
	}

	PolyVertIdx Loop::findConcavestPt() const
	{
		auto minIter = max_element(_concativty.begin(), _concativty.end());
		return PolyVertIdx(minIter - _concativty.begin());
	}

	void Loop::removeDuplicate()
	{
		// handle degeneration
		for (int i = 0; i < ptCount(); ++i)
		{
			int ni = (i + 1) % ptCount();
			cvVec2f v = _vertices[i];
			cvVec2f nv = _vertices[ni];

			if (nv.distance(v) < CV_FLOAT_EPS)
				_vertices.erase(_vertices.begin() + i);
		}
	}

	void Loop::initializeIn()
	{
		updateNormals();
		fixWinding();
		computeConcavity_in();
	}

	void Loop::initializeOut(const HullLoop& hull)
	{
		updateNormals();
		computePockets(hull);
		computeConcavity_out(hull);
	}

	void Loop::initializeAll(bool inner, const HullLoop& hull)
	{
		if (inner)
		{
			initializeIn();
		}
		else
		{
			initializeOut(hull);
		}
	}

	void Loop::computeCWPairs(bool inner)
	{
		if (inner)
		{
			_cwPair.first = PolyVertIdx(0);
			_cwPair.second = PolyVertIdx(ptCount() / 2 - 1);
		}
	}

	void Loop::findIntersections(const cvVec2f& p0, const cvVec2f& p1, vector<SegIntersect>& results)
	{
		const float eps = 1e-3f;
		cvVec2f d = p1 - p0;
		float len = d.length();
		results.clear();
		d /= len;

		if (len < CV_FLOAT_EPS) return;

		for (auto idx = beginIdx(); idx <= endIdx(); ++idx)
		{
			cvVec2f v0 = (*this)[idx];
			cvVec2f v1 = (*this)[nextIdx(idx)];

			float t;
			cvVec2f intersect;
			bool hit = IntersectRayLine(p0, d, v0, v1, t, intersect);
			float t1 = (intersect - p0).dot(d) / len;
			if(hit && (t1 > 0 && t1 < ( 1 - eps)))
			{
				SegIntersect inter;
				inter.clipp_t = t1;
				inter.subject_t = t;
				inter.intersect = intersect;
				inter.idx = idx;
				results.push_back(inter);
			}
		}

		sort(results.begin(), results.end());
	}

	bool Loop::clipLoop(const Loop& clip, const cvVec2f& clipPos, const HullLoop& hull, vector<Loop>& result)
	{
		vector<SegIntersect> allIntersects;
		vector<SegIntersect> segs;
		vector<PolyVertIdx> inPts;
		segs.reserve(2);
		SegIntersect lastInSec;
		PolyVertIdx lastInClipIdx(-1);
		SegIntersect lastOutSec;
		PolyVertIdx lastOutClipIdx(-1);

		bool hasOut = false;
		HullLoop::InOut inOut = hull.isPointInside(*this, clip[PolyVertIdx(0)] + clipPos);
		for (auto cIdx = clip.beginIdx(); cIdx <= clip.endIdx(); ++cIdx)
		{
			PolyVertIdx ncIdx = clip.nextIdx(cIdx);
			cvVec2f v = clip[cIdx] + clipPos;
			cvVec2f nv = clip[ncIdx] + clipPos;
			
			if (inOut == HullLoop::InOut::Out)
				hasOut = true;

			findIntersections(v, nv, segs);
			if (segs.size() == 2)
			{
				//cvAssert(inOut == HullLoop::InOut::Out);
				// cut through , one loop ready
				Loop nloop;
				nloop.AddVertex(segs[1].intersect);
				for(PolyVertIdx nidx = segs[1].idx; nidx != segs[0].idx; nidx = prevIdx(nidx))
				{
					nloop.AddVertex((*this)[nidx]);
				}
				nloop.AddVertex(segs[0].intersect);
				result.push_back(nloop);
			}
			else if (segs.size() == 1)
			{
				HullLoop::InOut nio = inOut == HullLoop::InOut::Out ? HullLoop::InOut::In : HullLoop::InOut::Out;

				if (nio == HullLoop::InOut::In)
				{
					if (lastOutSec.idx.val() == -1)
					{
						lastInSec = segs[0];
						lastInClipIdx = cIdx;
					}
					else
					{ 
						// another loop is ready
						Loop nloop;
						nloop.AddVertex(segs[0].intersect);

						//from in to out forward
						for (PolyVertIdx idx = nextIdx(segs[0].idx); idx != lastOutSec.idx; idx = nextIdx(idx))
						{
							nloop.AddVertex((*this)[idx]);
						}
						nloop.AddVertex((*this)[lastOutSec.idx]);
						nloop.AddVertex(lastOutSec.intersect);

						for (PolyVertIdx idx = lastOutClipIdx; idx != cIdx; idx = clip.prevIdx(idx))
						{
							nloop.AddVertex(clip[idx] + clipPos);
						}

						result.push_back(nloop);
					}

					if (nio != inOut)
					{
						lastInClipIdx = cIdx;
						lastInSec = segs[0];
					}
				}
				else
				{
					// we just got out
					if (lastInSec.idx.val() == -1) // no known outside  save this point
					{
						lastOutClipIdx = cIdx;
						lastOutSec = segs[0];
					}
					else
					{
						// an loop is ready
						Loop nloop;
						nloop.AddVertex(lastInSec.intersect);
						for (auto nidx = nextIdx(lastInSec.idx); nidx != segs[0].idx; nidx = nextIdx(nidx))
						{
							nloop.AddVertex((*this)[nidx]);
						}
						nloop.AddVertex((*this)[segs[0].idx]);
						nloop.AddVertex(segs[0].intersect);

						for (auto nidx = cIdx; nidx != lastInClipIdx; nidx = clip.prevIdx(nidx))
						{
							nloop.AddVertex(clip[nidx] + clipPos);
						}

						result.push_back(nloop);

						lastInSec = SegIntersect();
						lastInClipIdx = PolyVertIdx(-1);
						lastOutSec = SegIntersect();
						lastOutClipIdx = PolyVertIdx(-1);
					}
				}
				inOut = nio;
			}
		}

		if (result.size() == 0 && !hasOut)
		{
			return true;
		}
		return false;
	}

	Loop Loop::duplicate(const cvVec2f& pos) const 
	{
		Loop l;

		for(int i = 0; i < ptCount(); ++i)
			l.AddVertex(_vertices[i] + pos);

		l.initializeIn();

		return l;
	}

	void HullLoop::insertAfterIdx(PolyVertIdx after, PolyVertIdx idx)
	{
		auto iter = find(ptIndicies.begin(), ptIndicies.end(), after);
		cvAssert(iter != ptIndicies.end());
		ptIndicies.insert(iter + 1, idx);
	}

	HullLoop::InOut HullLoop::isPointInside(const Loop& loop, const cvVec2f& pt) const
	{
		cvAssert(ptIndicies.size() > 0);
		for(int i = 0; i < ptIndicies.size(); ++i)
		{
			PolyVertIdx vidx = ptIndicies[i];
			PolyVertIdx nvidx = ptIndicies[(i + 1) % ptIndicies.size()];

			cvVec2f v0 = loop[vidx];
			cvVec2f v1 = loop[nvidx];

			cvVec2f d = pt - v0;
			cvVec2f v1v0 = v1 - v0;
			float cross = d.cross(v1v0);
			if (abs(cross) < CV_FLOAT_EPS)
				return HullLoop::InOut::Edge;
			else if (cross < 0)
				return HullLoop::InOut::Out;
		}
		return HullLoop::InOut::In;
	}
	HullLoop::InOut HullLoop::isPointInside(const Loop& loop, const PolyVertIdx& ptIdx) const
	{
		cvVec2f pt = loop[ptIdx];
		return isPointInside(loop, pt);
	}

	void Polygon::computeHull()
	{
		_hull = acd::_quickHull(outterLoop());
	}

	WitnessPt Polygon::findWitnessPt() const
	{
		return acd::pickCW(*this, _hull, outterLoop().pockets());
	}

	void Polygon::initializeAll()
	{
		if (loops.size() > 0)
		{
			loops[0].fixWinding();
			loops[0].removeDuplicate();
		}

		computeHull();
		for (auto iter = loops.begin(); iter != loops.end(); ++iter)
		{
			(*iter).initializeAll(iter != loops.begin(), _hull);
		}
	}

	CutPoint Polygon::findBestCutPt(const WitnessPt& wp) const
	{
		return acd::findBestCutPt(*this, _hull, outterLoop().pockets(), wp);
	}

	bool Polygon::identical(const Polygon& other) const
	{
		if (other.loops.size() != loops.size())
			return false;

		const float eps = 1e-3f;
		for (int i = 0; i < loops.size(); ++i)
		{
			const Loop& l = loops[i];
			const Loop& ol = other.loops[i];

			if (ol.ptCount() != l.ptCount())
				return false;

			auto& ola = ol.getVertsArray();
			auto& la = l.getVertsArray();
			for (int i = 0; i < ola.size(); ++i)
			{
				if (abs(ola[i].x - la[i].x) > eps)
					return false;

				if (abs(ola[i].y - la[i].y) > eps)
					return false;
			}
		}
		return true;
	}

	float Polygon::area() const 
	{
		float outArea = outterLoop().area();
		cvAssert(outArea > 0);
		for (int i = 1; i < loops.size(); ++i)
		{
			outArea -= loops[i].area();
		}
		return outArea;
	}
}
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
		:_vertices(vtx)
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

	void Loop::computeConcavity_in(const HullLoop& hull)
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

	void Loop::initializeAll(bool inner, const HullLoop& hull)
	{
		updateNormals();
		if (inner)
		{
			fixWinding();
			computeConcavity_in(hull);
		}
		else
		{
			computePockets(hull);
			computeConcavity_out(hull);
		}
	}

	void HullLoop::insertAfterIdx(PolyVertIdx after, PolyVertIdx idx)
	{
		auto iter = find(ptIndicies.begin(), ptIndicies.end(), after);
		cvAssert(iter != ptIndicies.end());
		ptIndicies.insert(iter + 1, idx);
	}

	HullLoop::InOut HullLoop::isPointInside(const Loop& loop, const PolyVertIdx& ptIdx) const
	{
		cvVec2f pt = loop[ptIdx];
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
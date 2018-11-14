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

	void Loop::initializeAll(bool inner, const HullLoop& hull)
	{
		updateNormals();
		if (inner)
		{
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
		computeHull();
		for (auto iter = loops.begin(); iter != loops.end(); ++iter)
		{
			(*iter).initializeAll(iter != loops.begin(), _hull);
		}
	}

	PolyVertIdx Polygon::findBestCutPt(const WitnessPt& wp) const
	{
		return acd::findBestCutPt(*this, _hull, outterLoop().pockets(), wp);
	}
}
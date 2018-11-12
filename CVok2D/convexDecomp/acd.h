#pragma once
#include <core/cvColor.h>
#include <core/cvMath.h>
#include <vector>
#include <algorithm>
#include "acd_base.h"

namespace acd
{
	using namespace std;

	typedef IndexBase<int> HullIdx;
	typedef IndexBase<size_t> PolyVertIdx;

    class Loop
    {
	public:
		Loop() { }
		Loop(const vector<cvVec2f>& vtx);
		bool AreNeighbour(const PolyVertIdx& idx0, const PolyVertIdx& idx1) const
		{
			return AreNeighbour(idx0.val(), idx1.val());
		}
		PolyVertIdx prevIdx(const PolyVertIdx& idx) const { return PolyVertIdx(prevIdx(idx.val())); }
		PolyVertIdx nextIdx(const PolyVertIdx& idx) const { return PolyVertIdx(nextIdx(idx.val())); }
		cvVec2f operator[](const PolyVertIdx& idx)const { return _vertices[idx.val()]; }
		size_t ptCount() const { return _vertices.size(); }
		void AddVertex(const cvVec2f& vtx) { _vertices.push_back(vtx); updateNormals(); }

		// iterators
		typedef vector<cvVec2f>::iterator iterator;
		typedef vector<cvVec2f>::const_iterator const_iterator;
		iterator begin() { return _vertices.begin(); }
		const_iterator cbegin() const { return _vertices.begin(); }
		iterator end() { return _vertices.end(); }
		const_iterator cend() const { return _vertices.end(); }

		// indices
		PolyVertIdx beginIdx() const { return PolyVertIdx(0); }
		PolyVertIdx endIdx() const { return PolyVertIdx(_vertices.size() - 1); }

		//normals
		cvVec2f normal(PolyVertIdx idx) const { return _normals[idx.val()]; }

		// update functions
		void updateNormals();

		// is v1 precedent of v
		bool isPrev(PolyVertIdx v, PolyVertIdx v1) const { return (v1.val() + 1) % ptCount() == v.val(); }
		// is v1 successor of v
		bool isNext(PolyVertIdx v, PolyVertIdx v1) const { return (v.val() + 1) % ptCount() == v1.val(); }
	private:
		vector<cvVec2f> _vertices;
		vector<cvVec2f> _normals; //normal of each edge ( perpendicular to nextVert - curVert)
		bool AreNeighbour(size_t idx0, size_t idx1) const;
		size_t prevIdx(size_t idx) const { return idx == 0 ? _vertices.size() - 1 : idx - 1; }
		size_t nextIdx(size_t idx) const { return (idx + 1) % _vertices.size(); }
	};

	// a collection of indices into original polygon
	// that forms the convex hull
	class HullLoop
	{
	private:
		vector<PolyVertIdx> ptIndicies; //index into original polygon
	public:
		// operator overload
		PolyVertIdx operator[](const HullIdx& idx) const { return polyIdx(idx); };
		void addIndex(PolyVertIdx idx) { ptIndicies.push_back(idx); }
		void sort() { std::sort(ptIndicies.begin(), ptIndicies.end()); }
		void insertAfterIdx(PolyVertIdx after, PolyVertIdx idx);
		//const vector<int>& getPtIndices() const { return ptIndicies; }
		size_t pointCount() const { return ptIndicies.size(); }

		bool hasVtxIdx(PolyVertIdx idx)  const
		{
			return find(ptIndicies.begin(), ptIndicies.end(), idx) != ptIndicies.end();
		}

	public:
		typedef vector<PolyVertIdx>::iterator iterator;
		typedef vector<PolyVertIdx>::const_iterator const_iterator;
		iterator begin() { return ptIndicies.begin(); }
		const_iterator cbegin() const { return ptIndicies.begin(); }
		iterator end() { return ptIndicies.end(); }
		const_iterator cend() const { return ptIndicies.end(); }
	private:
		PolyVertIdx polyIdx(const HullIdx& hi) const { return ptIndicies[hi.val()]; }
	};

    class Polygon
    {
	public:
		vector<Loop> loops;
		Polygon()
		{
			loops.push_back(Loop());
		}
		void reset() { loops.clear(); }
	};

    class Bridge
    {
	public:
		Bridge() :idx0(0), idx1(0) {}
		// 2 points forming bridge
		PolyVertIdx idx0;		PolyVertIdx idx1;
		// indices of notches in original polygon 
		vector<PolyVertIdx> notches;
	};

    struct ConvexHull
    {
	public:
		Bridge bridge;
        Loop origLoop;

        vector<int> hullIndex;
	};

    struct WitnessPt
    {
	public:
		WitnessPt(): ptIndex(-1) {}
		float Concavity;
        int loopIndex;
        PolyVertIdx ptIndex; //index of witness point in original polygon
		int pocketIdx;
    };
	
	vector<Bridge> _findAllPockets(const HullLoop& hull, const Loop& loop);
	HullLoop _quickHull(const Loop& loop);
	WitnessPt _pickCW(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockes);
	PolyVertIdx _findBestCutPt(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockets,
						const WitnessPt& cwp);

	vector<Loop> _resolveLoop(const Loop& loop);

}
#pragma once
#include <core/cvColor.h>
#include <core/cvMath.h>
#include <vector>
#include <algorithm>

namespace acd
{
	using namespace std;

	struct HullIdx 
	{ 
	public:
		int idx; 

	public:
		HullIdx(int i) : idx(i) {}
		bool operator==(const HullIdx& o) const
		{
			return o.idx == idx;
		}
		bool operator<(const HullIdx& o) const { return idx < o.idx; }
		void operator=(const HullIdx& o) { idx = o.idx; }
	};

    class Loop
    {
	public:
		vector<cvVec2f> Vertices;
		bool AreNeighbour(int idx0, int idx1) const;
		size_t prevIdx(int idx) const { return idx == 0 ? Vertices.size() - 1 : idx - 1; }
		size_t nextIdx(int idx) const { return (idx + 1) % Vertices.size(); }
	};

	// a collection of indices into original polygon
	// that forms the convex hull
	class HullLoop
	{
	private:
		vector<int> ptIndicies; //index into original polygon
	public:
		// operator overload
		size_t operator[](const HullIdx& idx) const { return polyIdx(idx); };
		void addIndex(int idx) { ptIndicies.push_back(idx); }
		void sort() { std::sort(ptIndicies.begin(), ptIndicies.end()); }
		void insertAfterIdx(int after, int idx);
		//const vector<int>& getPtIndices() const { return ptIndicies; }
		size_t pointCount() const { return ptIndicies.size(); }
	private:
		size_t polyIdx(const HullIdx& hi) const { return ptIndicies[hi.idx]; }
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
        // 2 index of bridge points on convex hull 
	public:
		Bridge() :idx0(0), idx1(0) {}
		HullIdx idx0;
		HullIdx idx1;
		// indices of notches in original polygon 
		vector<int> notches;
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
		float Concavity;
        int loopIndex;
        int ptIndex; //index of witness point in original polygon
		int pocketIdx;
    };
	
	vector<Bridge> _findAllPockets(const HullLoop& hull, const Loop& loop);
	HullLoop _quickHull(const Loop& loop);
	WitnessPt _pickCW(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockes);
	int _findBestCutPt(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockets,
						const WitnessPt& cwp);

	vector<Polygon> _resolveLoop(const Loop& loop, const HullLoop& hull, const WitnessPt& cwp, 
		int cutPtIdx);

}
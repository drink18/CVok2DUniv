#pragma once
#include <core/cvColor.h>
#include <core/cvMath.h>
#include <vector>
#include <algorithm>

namespace acd
{
	using namespace std;
    class Loop
    {
	public:
		vector<cvVec2f> Vertices;
		bool AreNeighbour(int idx0, int idx1) const;
		size_t prevIdx(int idx) const { return idx == 0 ? Vertices.size() - 1 : idx - 1; }
		size_t nextIdx(int idx) const { return (idx + 1) % Vertices.size(); }
	};
	class HullLoop
	{
	private:
		vector<int> ptIndicies;
	public:
		void addIndex(int idx) { ptIndicies.push_back(idx); }
		void sort() { std::sort(ptIndicies.begin(), ptIndicies.end()); }
		void insertAfterIdx(int after, int idx);
		const vector<int>& getPtIndices() const { return ptIndicies; }
	};

    class Polygon
    {
	public:
		vector<cvVec2f> Pathes;
	};

    class Bridge
    {
        // 2 index of vertices on original loop that forms bridge;
	public:
		int idx0;
        int idx1;
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
        int ptIndex; //index of witness point in loop
		int pocketIdx;
    };
	
	vector<Bridge> _findAllPockets(const HullLoop& hull, const Loop& loop);
	HullLoop _quickHull(const Loop& loop);
	WitnessPt _pickCW(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockes);
	int _findBestCutPt(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockets,
						const WitnessPt& cwp);

}
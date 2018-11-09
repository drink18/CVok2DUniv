#pragma once
#include <core/cvColor.h>
#include <core/cvMath.h>
#include <vector>

namespace acd
{
	using namespace std;
    class Loop
    {
	public:
		vector<cvVec2f> Vertices;
		bool AreNeighbour(int idx0, int idx1) const;
		int prevIdx(int idx) const { return idx == 0 ? Vertices.size() - 1 : idx - 1; }
		int nextIdx(int idx) const { return (idx + 1) % Vertices.size(); }
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
        Loop loop;
        int loopIndex;
        int ptIndex; //index of witness point in loop

        WitnessPt(float _concavity, Loop _loop, int _loopIndex, int _ptIndex)
        {
            Concavity = _concavity;
            loop = _loop;
            loopIndex = _loopIndex;
            ptIndex = _ptIndex;
        }
    };
	
	vector<Bridge> _findAllPockets(const vector<int>& hull, const Loop& loop);
	vector<int> _quickHull(const Loop& loop);
}
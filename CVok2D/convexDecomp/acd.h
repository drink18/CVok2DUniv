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
       vector<cvVec2f> notches;
	};

    struct ConvexHull
    {
	public:
		Bridge bridge;
        Loop origLoop;
        vector<int> hullIndex;
	};
	
	vector<int> _quickHull(Loop loop);
}
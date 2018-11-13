#pragma once
#include <vector>
#include <algorithm>
#include "acd_base.h"

namespace acd
{
	using namespace std;

	
	vector<Bridge> _findAllPockets(const HullLoop& hull, const Loop& loop);
	HullLoop _quickHull(const Loop& loop);
	WitnessPt _pickCW(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockes);
	PolyVertIdx _findBestCutPt(const Loop& loop, const HullLoop& hull, const vector<Bridge>& pockets,
						const WitnessPt& cwp);

	vector<Polygon> _resolveLoop(const Polygon& polygon);
	CutLine _findCutLine(const Loop& loop, PolyVertIdx origin);

	Winding _testWinding(const vector<cvVec2f>& verts);

}
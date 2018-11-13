#pragma once
#include <vector>
#include <algorithm>
#include "acd_base.h"

namespace acd
{
	using namespace std;

	
	vector<Bridge> findAllPockets(const HullLoop& hull, const Loop& loop);
	HullLoop quickHull(const Polygon& polygon);
	WitnessPt pickCW(const Polygon& poly, const HullLoop& h, const vector<Bridge>& pockets);
	PolyVertIdx findBestCutPt(const Polygon& polygon, const HullLoop& hull, const vector<Bridge>& pockets,
		const WitnessPt& cwp);

	vector<Polygon> _resolveLoop(const Polygon& polygon);
	CutLine findCutLine(const Polygon& polygon, const WitnessPt& wp);

	Winding _testWinding(const vector<cvVec2f>& verts);

}
#pragma once
#include <vector>
#include <algorithm>
#include "acd_base.h"

namespace acd
{
	using namespace std;

	
	vector<Pocket> findAllPockets(const HullLoop& hull, const Loop& loop);
	HullLoop quickHull(const Polygon& polygon);
	WitnessPt pickCW(const Polygon& poly, const HullLoop& h, const vector<Pocket>& pockets);
	PolyVertIdx findBestCutPt(const Polygon& polygon, const HullLoop& hull, const vector<Pocket>& pockets,
		const WitnessPt& cwp);

	vector<Polygon> _resolveLoop(const Polygon& polygon);
	CutLine findCutLine(const Polygon& polygon, const WitnessPt& wp);

	HullLoop _quickHull(const Loop& loop);
	Winding _testWinding(const vector<cvVec2f>& verts);
	float _computeConcavity_out(const Loop& loop, const Pocket& pocket, int notchIdx);
}
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
	CutPoint findBestCutPt(const Polygon& polygon, const HullLoop& hull, const vector<Pocket>& pockets,
		const WitnessPt& cwp);

	vector<Polygon> _resolveLoop_OneStep(const Polygon& polygon);
	vector<Polygon> _resolveLoop_All(const Polygon& polygon);
	vector<Polygon> _resolvePolyList(const vector<Polygon>& polygons);
	CutLine findCutLine(const Polygon& polygon, const WitnessPt& wp);

	HullLoop _quickHull(const Loop& loop);
	Winding _testWinding(const vector<cvVec2f>& verts);
	float _computeConcavity_out(const Loop& loop, const Pocket& pocket, int notchIdx);
	float _computeConcavity_in(const Loop& loop, const Loop& outLoop);

	void writePolygonInfo(ostream& stream, const Polygon& poly);
	void writePolygonListInfo(ostream& stream, const vector<Polygon>& poly);
	vector<Polygon> readPolygon(ifstream& stream);

	//utils
	bool IntersectRayLine(cvVec2f o, cvVec2f dir, cvVec2f from, cvVec2f to, float& outT, cvVec2f& intersect);

	//debugging stuff
	Loop _makeRoundLoop(const cvVec2f& center, float radius, int nbSeg, float rotation);
}
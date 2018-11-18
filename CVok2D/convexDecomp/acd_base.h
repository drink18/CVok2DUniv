#pragma  once

#include <vector>
#include <algorithm>
#include <core/cvColor.h>
#include <core/cvMath.h>


namespace acd
{
	using namespace std;

	class Loop;
	class HullLoop;

	template<typename VTYPE, VTYPE INVALID_VAL>
	struct IndexBase
	{
	private:
		VTYPE idx;
	public:
		typedef IndexBase<VTYPE, INVALID_VAL>  SelfType;
		explicit IndexBase(VTYPE _v) : idx(_v) {}
		IndexBase(const SelfType& o) : idx(o.idx) {}
		bool operator!=(const SelfType& o) const { return o.idx != idx; }
		bool operator==(const SelfType& o) const { return o.idx == idx; }
		bool operator< (const SelfType& o) const { return idx < o.idx; }
		bool operator<= (const SelfType& o) const { return idx <= o.idx; }
		bool operator>(const SelfType& o) const { return idx > o.idx; }
		SelfType operator+(const SelfType& o) const { return SelfType(idx + o.idx); }
		SelfType operator+(VTYPE inc) const { return SelfType(idx + inc); }
		SelfType operator++(int) { idx++; return *this; }
		SelfType operator--(int) { idx--; return *this; }
		SelfType& operator++() { idx++; return *this; }
		SelfType& operator--() { idx--; return *this; }
		VTYPE val() const { return idx; }
		bool isValid() const { return idx != INVALID_VAL; }
	};

	typedef IndexBase<size_t, -1> HullIdx;
	typedef IndexBase<size_t, -1> PolyVertIdx;
	typedef IndexBase<size_t, -1> LoopIdx;

	enum class Winding : uint8_t
	{
		CCW,
		CW
	};

	struct SegIntersect
	{
		SegIntersect() : idx(-1) {}
		cvVec2f intersect;
		PolyVertIdx idx;
		float subject_t;
		float clipp_t;
		bool operator<(const SegIntersect& o) const {return clipp_t < o.clipp_t; }
	};

	struct CutPoint
	{
		cvVec2f point;
		PolyVertIdx idx;
		PolyVertIdx prevIdx;
		PolyVertIdx nextIdx;
		CutPoint()
			:idx(-1),
			prevIdx(-1),
			nextIdx(-1)
		{ }
	};

	struct WitnessPt
	{
	public:
		WitnessPt() : ptIndex(-1) {}
		float Concavity;
		int loopIndex;
		PolyVertIdx ptIndex; //index of witness point in original polygon
		int pocketIdx;

		bool onInnerLoop() const { return loopIndex > 0; }
	};

	struct CutLine
	{
	public:
		CutLine()  {}
		WitnessPt orgin;
		cvVec2f originPt;
		cvVec2f lineDir;
	};

	float _polyArea(const vector<cvVec2f>& verts);

	// a collection of indices into original polygon
	// that forms the convex hull
	class HullLoop
	{
	private:
		vector<PolyVertIdx> ptIndicies; //index into original polygon
	public:
		enum class InOut 
		{
			In,
			Out,
			Edge
		};
	public:
		HullLoop() {}
		HullLoop(const HullLoop& other)
			: ptIndicies(other.ptIndicies) { }

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
	public:
		HullLoop::InOut isPointInside(const Loop& loop, const PolyVertIdx& ptIdx) const;
		HullLoop::InOut isPointInside(const Loop& loop, const cvVec2f& pt) const;
	private:
		PolyVertIdx polyIdx(const HullIdx& hi) const { return ptIndicies[hi.val()]; }
	};

	class Pocket
	{
	public:
		Pocket() :idx0(0), idx1(0) {}
		// 2 points forming bridge
		PolyVertIdx idx0;
		PolyVertIdx idx1;
		// indices of notches in original polygon 
		vector<PolyVertIdx> notches;
	};

	class Loop
	{
	public:
		Loop() : _cwPair(PolyVertIdx(-1), PolyVertIdx(-1)) { }
		Loop(const vector<cvVec2f>& vtx);
		bool AreNeighbour(const PolyVertIdx& idx0, const PolyVertIdx& idx1) const
		{
			return AreNeighbour(idx0.val(), idx1.val());
		}
		PolyVertIdx prevIdx(const PolyVertIdx& idx) const { return PolyVertIdx(prevIdx(idx.val())); }
		PolyVertIdx nextIdx(const PolyVertIdx& idx) const { return PolyVertIdx(nextIdx(idx.val())); }
		cvVec2f operator[](const PolyVertIdx& idx)const { return _vertices[idx.val()]; }
		size_t ptCount() const { return _vertices.size(); }
		void AddVertex(const cvVec2f& vtx) { _vertices.push_back(vtx); }

		void fixWinding();

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
		void computeConcavity_in();
		void computeConcavity_out(const HullLoop& hullLoop);
		void computePockets(const HullLoop& hull);
		void initializeAll(bool inner, const HullLoop& hull);
		void initializeIn();
		void initializeOut(const HullLoop& hull);
		void removeDuplicate();
		void computeCWPairs(bool inner);

		// queries
		float area() const { return _polyArea(_vertices); }
		void findIntersections(const cvVec2f& p0, const cvVec2f& p1, vector<SegIntersect>& results);

		// clipping 
		bool clipLoop(const Loop& clip, const cvVec2f& clipPos, const HullLoop& hull, vector<Loop>& result);

		//concavity points related
		PolyVertIdx findConcavestPt() const;


		// neighborhood tools
		// is v1 precedent of v
		bool isPrev(PolyVertIdx v, PolyVertIdx v1) const { return (v1.val() + 1) % ptCount() == v.val(); }
		// is v1 successor of v
		bool isNext(PolyVertIdx v, PolyVertIdx v1) const { return (v.val() + 1) % ptCount() == v1.val(); }

		// accessors (TODO : HIDE)
		const vector<cvVec2f>& getVertsArray() const { return _vertices; }
		const vector<float>& concavity() const { return _concativty; }
		const vector<Pocket>& pockets() const { return _pockets; }

		// miscellaneous 
	public:
		Loop duplicate(const cvVec2f& pos) const;
	private:
		vector<cvVec2f> _vertices;
		vector<cvVec2f> _normals; //normal of each edge ( perpendicular to nextVert - curVert)
		vector<float> _concativty; //concavtiy
		vector<Pocket> _pockets;

		bool AreNeighbour(size_t idx0, size_t idx1) const;
		size_t prevIdx(size_t idx) const { return idx == 0 ? _vertices.size() - 1 : idx - 1; }
		size_t nextIdx(size_t idx) const { return (idx + 1) % _vertices.size(); }

		pair<PolyVertIdx, PolyVertIdx> _cwPair;
	};


	class Polygon
	{
	public:
		vector<Loop> loops;
		Polygon()
		{
		}
		void reset() { loops.clear(); }

		Loop& outterLoop() 
		{
			if (loops.empty())
				loops.push_back(Loop());
			return loops[0];
		}
		const Loop& outterLoop() const { return loops[0]; }
		bool hasHole() const { return loops.size() > 1; }
		void addLoop(const Loop& loop) { loops.push_back(loop); }
	public:
		// iterators
		typedef vector<Loop>::iterator iterator;
		typedef vector<Loop>::const_iterator const_iterator;
		iterator begin() { return loops.begin(); }
		const_iterator cbegin() const { return loops.begin(); }
		iterator end() { return loops.end(); }
		const_iterator cend() const { return loops.end(); }

		//overridden operators
		bool operator==(const Polygon& other)const { return identical(other); }
		bool operator!=(const Polygon& other)const { return !identical(other); }

		//init
		void computeHull();
		void initializeAll();

		//concavity points related
		WitnessPt findWitnessPt() const;

		//accessor 
		const HullLoop& convexHull() const { return _hull; }


		// helpers
		bool isConvex() const{ return loops.size() == 1 && _hull.pointCount() == outterLoop().ptCount(); }
		bool identical(const Polygon& other) const;

		// cutting
		CutPoint findBestCutPt(const WitnessPt& wp) const;

		float area() const;

	private:
		HullLoop _hull;
	};

	

	struct ConvexHull
	{
	public:
		Pocket bridge;
		Loop origLoop;

		vector<int> hullIndex;
	};

}

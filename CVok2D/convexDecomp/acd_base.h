#pragma  once

template<typename VTYPE>
struct IndexBase
{
private:
	VTYPE idx;
public:
	IndexBase(VTYPE _v) : idx(_v) {}
	IndexBase(const IndexBase<VTYPE>& o) : idx(o.idx) {}
	bool operator==(const IndexBase<VTYPE>& o) const { return o.idx == idx; }
	bool operator< (const IndexBase<VTYPE>& o) const { return idx < o.idx; }
	VTYPE val() const { return idx; }
};

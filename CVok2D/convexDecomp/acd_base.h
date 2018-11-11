#pragma  once

template<typename VTYPE>
struct IndexBase
{
private:
	VTYPE idx;
public:
	explicit IndexBase(VTYPE _v) : idx(_v) {}
	IndexBase(const IndexBase<VTYPE>& o) : idx(o.idx) {}
	bool operator==(const IndexBase<VTYPE>& o) const { return o.idx == idx; }
	bool operator< (const IndexBase<VTYPE>& o) const { return idx < o.idx; }
	bool operator<= (const IndexBase<VTYPE>& o) const { return idx <= o.idx; }
	bool operator>(const IndexBase<VTYPE>& o) const { return idx > o.idx; }
	IndexBase<VTYPE> operator+(const IndexBase<VTYPE>& o) const { return IndexBase<VTYPE>(idx + o.idx); }
	IndexBase<VTYPE> operator+(VTYPE inc) const { return IndexBase<VTYPE>(idx + inc); }
	IndexBase<VTYPE> operator++(int) { idx++; return *this; }
	IndexBase<VTYPE> operator--(int) { idx--; return *this; }
	IndexBase<VTYPE>& operator++() { idx++; return *this; }
	IndexBase<VTYPE>& operator--() { idx--; return *this; }
	VTYPE val() const { return idx; }
};

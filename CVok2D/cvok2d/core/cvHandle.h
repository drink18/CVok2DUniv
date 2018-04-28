#pragma once
#include <functional>

template<typename T, T INVALID_VALUE> 
struct cvHandle
{
public:
	cvHandle() : m_val(INVALID_VALUE) {}
	cvHandle(T val) : m_val(val) {}
	T getVal() const { return m_val; }
	bool isValid() const { return m_val != INVALID_VALUE;	}
	static T invalid() { return INVALID_VALUE; }

	cvHandle& operator=(const cvHandle& other) { m_val = other.m_val; return *this; }

	bool operator==(const cvHandle<T, INVALID_VALUE>& other)  const
	{return  m_val == other.m_val; }

    bool operator!=(const cvHandle<T, INVALID_VALUE>& other) const
    { return m_val != other.m_val; }

private:
	T m_val;
};

namespace std
{
    template<typename  T, T inv>
    struct hash<cvHandle<T, inv>>
    {
        size_t operator() (const cvHandle<T, inv>& h) const
        {
            hash<uint16_t> h16;
            return h16(h.getVal());
        }
    };
}

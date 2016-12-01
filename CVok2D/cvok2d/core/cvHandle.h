#pragma once

template<typename T, T INVALID_VALUE> 
struct cvHandle
{
public:
	cvHandle() : m_val(INVALID_VALUE) {}
	cvHandle(T val) : m_val(val) {}
	T getVal() const { return m_val; }
	bool isValid() const { return m_val != INVALID_VALUE;	}
	static T invalid() { return INVALID_VALUE; }

	cvHandle& operator=(cvHandle& other) { m_val = other.m_val; return *this; }

	bool operator==(const cvHandle<T, INVALID_VALUE>& other)  const
	{return  m_val == other.m_val; }
private:
	T m_val;
};

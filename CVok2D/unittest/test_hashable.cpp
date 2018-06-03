#include <gtest/gtest.h>
#include <collision/cvManifold.h>

#include <memory>
#include <unordered_set>

//hashable struct

using namespace std;
TEST(NPPair, HashNPPair)
{
	unordered_set<cvNPPair> pairSet;
	cvNPPair p1;

	ASSERT_TRUE(pairSet.find(p1) == pairSet.end());
	p1.m_bodyA = cvBodyId(0);
	p1.m_bodyB = cvBodyId(1);
	pairSet.insert(p1);

	ASSERT_TRUE(pairSet.find(p1) != pairSet.end());

	pairSet.erase(p1);

	ASSERT_TRUE(pairSet.find(p1) == pairSet.end());
}
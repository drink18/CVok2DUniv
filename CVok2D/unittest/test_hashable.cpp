#include <gtest/gtest.h>
#include <collision/cvManifold.h>

#include <memory>
#include <unordered_set>

//hashable struct

using namespace std;
TEST(NPPair, HashNPPair)
{
	unordered_set<cvNPPair> pairSet;
	cvNPPair p1(cvBodyId(0), cvBodyId(1));

	ASSERT_TRUE(pairSet.find(p1) == pairSet.end());
	pairSet.insert(p1);

	ASSERT_TRUE(pairSet.find(p1) != pairSet.end());

	pairSet.erase(p1);

	ASSERT_TRUE(pairSet.find(p1) == pairSet.end());
}

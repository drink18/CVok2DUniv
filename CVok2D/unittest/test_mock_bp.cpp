#include "gmock/gmock.h"

#include <memory>
#include <world/cvBroadphase.h>
#include <world/cvWorld.h>

class cvBroadphaseMock : public cvBroadphase
{
public:
    cvBroadphaseMock(){}
    //cvBroadphaseMock(const cvBroadphaseCInfo& info) : cvBroadphase(info) {}
    MOCK_METHOD3(updateDirtyNodes, void(cvWorld& world, std::vector<BPPair>& newPairs, std::vector<BPPair>& deletedPairs));
    MOCK_METHOD2(updateOneNode, void(cvBroadphaseHandle handle, const cvAabb& newAabb));
	MOCK_METHOD1(addNode, cvBroadphaseHandle(const cvAabb& nodeAabb));
	MOCK_METHOD1(removeNode, void(cvBroadphaseHandle handle));
	MOCK_METHOD1(getAllPairs, void(std::vector<BPPair>& pairs));

    MOCK_METHOD1(addBody, void(cvBody& body));
    MOCK_METHOD1(removeBody, void(cvBody& body));
    MOCK_METHOD1(markBodyDirty, void(const cvBody& body));

    MOCK_METHOD2(addPair, bool (const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2));
    MOCK_METHOD2(removePair, bool (const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2));
};


TEST(BPMock, bpMock)
{
    cvWorldCInfo wldInfo;
    cvBroadphaseCInfo bpInfo;
    //wldInfo.m_broadPhase = new cvBroadphaseMock();
}

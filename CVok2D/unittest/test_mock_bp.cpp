#include "gmock/gmock.h"
#include <simulation/cvBroadphase.h>

class cvBroadphaseMock : public cvBroadphase
{
    MOCK_METHOD1(updateDirtyNodes, void(cvWorld& world));
    MOCK_METHOD2(updateDirtyNodes, void(cvBroadphaseHandle handle, const cvAabb& newAabb));
	MOCK_METHOD1(addNode, cvBroadphaseHandle(const cvAabb& nodeAabb));
	MOCK_METHOD1(removeNode, void(cvBroadphaseHandle handle));
	MOCK_METHOD1(getAllPairs, void(std::vector<BPPair>& pairs));

    MOCK_METHOD1(addBody, void(cvBody& body));
    MOCK_METHOD1(removeBody, void(cvBody& body));
    MOCK_METHOD1(markBodyDirty, void(cvBody& body));

    MOCK_METHOD2(addPair, bool (const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2));
    MOCK_METHOD2(removePair, bool (const cvBroadphaseHandle& handle1, const cvBroadphaseHandle& handle2));
};

#include <gtest/gtest.h>
#include <cvok2dInit.h>
#include <collision/cvCollisionDispatch.h>

TEST(cvTestInit, colDispatch)
{
    cv2DInit();
    for(int i = 0; i < cvShape::eShapeType_Count; ++i)
    {
        for(int j = i; j < cvShape::eShapeType_Count; ++j)
        {
            EXPECT_NE(nullptr, g_collisionFunction[i][j]);
        }
    }
}

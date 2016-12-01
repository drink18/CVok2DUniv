#include "stdafx.h"
#include <gtest/gtest.h>

#include <cvok2d/core/collection/cvFreeList.h>
#include <cvok2d/core/cvHandle.h>

TEST(TestFreeList, addAndFree)
{
	struct DummyNode
	{
		int index;
		int crap;
	};

	typedef cvHandle<int32_t, 0x7fffffff> Handle;
	cvFreeList<DummyNode, Handle> dummyFreeList;
	
	const int initHandles = 100;
	Handle handles[initHandles];
	// insert 100 node
	for (int i = 0; i < initHandles; i++)
	{
		handles[i]= dummyFreeList.alloc();
		DummyNode& node = dummyFreeList.getAt(handles[i]);
		node.index = i;
		EXPECT_TRUE(handles[i].isValid());
	}

	// free 50, 51, 52
	dummyFreeList.free(handles[50]);
	dummyFreeList.free(handles[51]);
	dummyFreeList.free(handles[52]);

	// alloc again, should get 52, 51, 50
	{
		Handle h = dummyFreeList.alloc();
		EXPECT_TRUE(h.getVal() == 52);
	}

	{
		Handle h = dummyFreeList.alloc();
		EXPECT_TRUE(h.getVal() == 51);
	}

	{
		Handle h = dummyFreeList.alloc();
		EXPECT_TRUE(h.getVal() == 50);
	}
}
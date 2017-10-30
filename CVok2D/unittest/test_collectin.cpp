#include <gtest/gtest.h>

#include <unordered_set>
#include <core/collection/cvFreeList.h>
#include <core/cvHandle.h>

using namespace std;
struct DummyNode
{
    int index;
    int crap;

    DummyNode()
    {
        index = 0;
        crap = 0;
    }
    DummyNode(int i, int val)
    {
        index = i;
        crap = val;
    }
};

typedef cvHandle<int32_t, 0x7fffffff> Handle;
namespace std
{
    template<> struct hash<Handle>
    {
        std::size_t operator()(const Handle& h) const
        {
            return hash{}(h.getVal());
        }
    };
}

TEST(TestFreeList, addAndFree)
{

	cvFreeList<DummyNode, Handle> dummyFreeList;

    unordered_set<Handle> handleset;
	const int initHandles = 100;
	Handle handles[initHandles];
	// insert 100 node
	for (int i = 0; i < initHandles; i++)
	{
		handles[i] = dummyFreeList.alloc(i, i);
		DummyNode& node = dummyFreeList.getAt(handles[i]);
		EXPECT_TRUE(handles[i].isValid());
	}

	// free 50, 51, 52
	dummyFreeList.free(handles[50]);
	dummyFreeList.free(handles[51]);
	dummyFreeList.free(handles[52]);

	// alloc again, should get 52, 51, 50
	{
		Handle h = dummyFreeList.alloc();
		EXPECT_EQ(52, h.getVal());
	}

	{
		Handle h = dummyFreeList.alloc();
		EXPECT_EQ(51, h.getVal());
	}

	{
		Handle h = dummyFreeList.alloc();
		EXPECT_EQ(50, h.getVal());
	}
}

TEST(TestFreeList, Constructor_Invok)
{
	typedef cvHandle<int32_t, 0x7fffffff> Handle;
	cvFreeList<DummyNode, Handle> dummyFreeList;

    Handle id = dummyFreeList.alloc(12, 13);

    DummyNode& node = dummyFreeList.getAt(id);
    EXPECT_EQ(node.index, 12);
}


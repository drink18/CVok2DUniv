#include <cvok2d.h>
#include <string>
#include <functional>
#include <vector>

#pragma once

class cvDebugDraw;
class cvWorld;

class TestBase
{
public:
    TestBase ();

    virtual void tick(cvDebugDraw& dbgDraw, float dt);
};

struct TestInfo
{
    typedef std::function<TestBase*()> TestCreateFn;

    TestCreateFn m_testFn;
    const char* m_name;

    TestInfo(const char* name, TestCreateFn fn)
        :m_name(name), m_testFn(fn) {}
};

extern std::vector<TestInfo>* g_RegisteredTests;
inline std::vector<TestInfo>& GetRegisteredTests()
{
    if(g_RegisteredTests == nullptr)
    {
        g_RegisteredTests = new std::vector<TestInfo>();
    }

    return *g_RegisteredTests;
}

struct RegisterTests
{
    RegisterTests(const char* name, TestInfo::TestCreateFn createFn)
    {
        TestInfo info(name, createFn);
        GetRegisteredTests().push_back(info);
    }
};

#define REGISTER_TEST(test_name, fn) \
    static RegisterTests t(test_name, fn);


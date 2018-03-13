#pragma  once

#include <cinttypes>
#include <core/cvHandle.h>

typedef cvHandle<std::uint16_t, 0x7FFF> cvBodyId;
typedef cvHandle<int32_t, (int32_t)0x7fffffff> cvBroadphaseHandle;

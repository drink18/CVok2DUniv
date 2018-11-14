#pragma once

#include "acd.h"
namespace crashHanlder
{
	void InstallCrashHandler();
	void AbortHandler(int signal);
}

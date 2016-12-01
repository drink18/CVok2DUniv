#pragma once
#include "cvBroadphase.h"

struct cvWorldCInfo
{

};

class cvWorld
{
public:
	cvWorld(cvWorldCInfo&  cinfo)
	{}
	
private:
	cvBroadphase* m_broadPhase;

};
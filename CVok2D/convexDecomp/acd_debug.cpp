#include "acd_base.h"
#include "acd.h"
#include <core/cvMath.h>


namespace acd
{
	Loop _makeRoundLoop(const cvVec2f& center, float radius, int nbSeg, float rotation)
	{
		Loop l;

		float a = rotation;
		float da = -2 * CV_PI / nbSeg;
		for (int i = 0; i < nbSeg; ++i)
		{
			float ca = a + da * i;
			cvVec2f v(cos(ca) * radius, sin(ca) * radius);
			l.AddVertex(v + center);
		}
		HullLoop hull;
		l.initializeAll(true, hull);

		return l;
	}
}
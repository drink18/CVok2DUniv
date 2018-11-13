#include "cvColor.h"
#include <cstdlib>

cvColorf cvColorf::Red(1.0f, 0, 0, 1);
cvColorf cvColorf::Blue(0.0f, 0, 1.0f, 1);
cvColorf cvColorf::Green(0.0f, 1.0f, 0, 1);
cvColorf cvColorf::White(1.0f, 1.0f, 1.0f, 1);
cvColorf cvColorf::Gray(0.5, 0.5f, 0.5f, 1);
cvColorf cvColorf::Purple(1.0f, 0.0f, 1.0f, 1);
cvColorf cvColorf::DarkPurple(0.2f, 0.137f, 0.329f);
cvColorf cvColorf::Yellow(1.0f, 1.0f, 0, 1);
cvColorf cvColorf::Cyan(0, 1.0f, 1.0f, 1);
cvColorf cvColorf::Black(0, 0, 0, 1);
cvColorf cvColorf::Orange(1, 0.644f, 0, 1);

cvColorf cvColorf::randomColor()
{
	float r = (float)rand() / RAND_MAX;
	float b = (float)rand() / RAND_MAX;
	float g = (float)rand() / RAND_MAX;
	return cvColorf(r, g , b, 1.0f);
} 
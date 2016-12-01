#pragma once

struct cvColorf
{
	cvColorf() {}
	cvColorf(float rIn, float gIn, float bIn, float aIn = 1.0f)
	{
		r = rIn; g = gIn; b = bIn; a = aIn;
	}

	void Set(float rIn, float gIn, float bIn, float aIn = 1.0f)
	{
		r = rIn; g = gIn; b = bIn; a = aIn;
	}

	float r, g, b, a;
};
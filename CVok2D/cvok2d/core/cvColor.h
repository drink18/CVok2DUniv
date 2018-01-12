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

    static cvColorf Red;
    static cvColorf Blue;
    static cvColorf Green;
    static cvColorf White;
    static cvColorf Gray;
    static cvColorf Purple;
    static cvColorf Yellow;
    static cvColorf Cyan;
};

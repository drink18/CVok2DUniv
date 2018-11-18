#include <gtest/gtest.h>
#include <acd.h>
#include <fstream>
#include <string>

using namespace acd;
using namespace std;

vector<Polygon> ResolvePolygon(const vector<Polygon>& poly)
{
	vector<Polygon> doneList;
	for (auto& p : poly)
	{
		auto res = acd::_resolveLoop_All(p);
		for (int ip = 0; ip < res.size(); ++ip)
		{
			doneList.push_back(res[ip]);
		}
	}
	return doneList;
}
void compareWithRef(ifstream& listFs)
{
	string testFileName;
	while (listFs >> testFileName)
	{
		string oname = string("out_" + testFileName);
		ifstream testFs(testFileName);
		ifstream outputFs(oname);

		vector<Polygon> inputList = acd::readPolygon(testFs);
		vector<Polygon> doneList = ResolvePolygon(inputList);
		vector<Polygon> ref = acd::readPolygon(outputFs);


		// compare area
		float origArea = 0;
		float resultArea = 0;
		for (auto& p : inputList)
			origArea += p.area();
		for (auto& p : doneList)
			resultArea += p.area();

		const float areaTolerance = 1e-3f;

		bool identical = true;

		if (ref.size() != doneList.size())
			identical = false;

		if (identical)
		{
			for (int i = 0; i < ref.size(); ++i)
			{
				Polygon& p = doneList[i];
				Polygon& refP = ref[i];

				if (p != refP)
					identical = false;
			}
		}

		EXPECT_NEAR(origArea, resultArea, areaTolerance)
			<< " area before/after decomposition mismatch for " << testFileName;

		EXPECT_TRUE(identical) << "Topology mismatch for " << testFileName;
		
	}
}


string g_listFile;

TEST(acdConvexDecomp, Preset_Cases)
{
	ifstream listFs(g_listFile);
	if (!listFs.is_open())
	{
		cerr << "Can't open test list file, skipping" << endl;
		return;
	}
	compareWithRef(listFs);
}

int main(int nargs, char** args)
{
	testing::InitGoogleTest(&nargs, args);

	if (nargs == 2) { g_listFile = args[1]; }

	return RUN_ALL_TESTS(); 
}

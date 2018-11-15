#include <acd.h>
#include <iostream>
#include <string>
#include <fstream>

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

int main(int narg, char** args)
{
    string listFile;
	if (narg < 2)
	{
		cout << u8"usage : (1) convexDecompCli.exe <testList>" << endl;
		cout << u8"usage : (2) convexDecompCli.exe runTest <testList>" << endl;
		return 1;
	}

	listFile = args[narg - 1];
	bool runTest = false;
	if (narg == 3) runTest = true;

	ifstream listFs(listFile);
	if (!listFs.is_open())
	{
		cerr << "Can't open test list file" << endl;
		return 1;
	}

	if(runTest)
	{ 
		//comparing
		string testFileName;
		while (listFs >> testFileName)
		{
			string oname = string("out_" + testFileName);
			ifstream testFs(testFileName);
			ifstream outputFs(oname);
			cout << "comparing " <<  testFileName  << " with " << oname << "........." ;
			
			vector<Polygon> poly = acd::readPolygon(testFs);
			vector<Polygon> doneList = ResolvePolygon(poly);

			vector<Polygon> ref = acd::readPolygon(outputFs);
			bool identical = true;

			if (ref.size() != doneList.size())
				identical = false;

			if(identical)
			{
				for (int i = 0; i < ref.size(); ++i)
				{
					Polygon& p = doneList[i];
					Polygon& refP = ref[i];

					if (p != refP)
						identical = false;
				}
			}

			if (identical)
				cout << "[Passed]" << endl;
			else
				cerr << "[Failed]" << endl;
		}
	}
	else
	{

		string testFileName;
		while (listFs >> testFileName)
		{
			cout << "filename= " << testFileName << endl;;
			string oname = string("out_" + testFileName);
			ifstream testFs(testFileName);
			vector<Polygon> poly = acd::readPolygon(testFs);
			vector<Polygon> doneList = ResolvePolygon(poly);


			ofstream ofs(oname, ios::out);
			writePolygonListInfo(ofs, doneList);
		}
	}

    return 0;
}

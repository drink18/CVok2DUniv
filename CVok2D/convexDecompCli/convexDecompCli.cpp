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

void compareWithRef(ifstream& listFs)
{
	string testFileName;
	while (listFs >> testFileName)
	{
		string oname = string("out_" + testFileName);
		ifstream testFs(testFileName);
		ifstream outputFs(oname);
		cout << "comparing " << testFileName << " with " << oname << "........." << endl;

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
		if (abs(origArea - resultArea) > areaTolerance)
		{
			cerr << "\t[Failed]:" << "area mismatch!"
				<< "\t\tinput area = " << origArea << ", result area = " << resultArea << endl;
		}
		else
		{
			cout << "\t[Passed]: area match!" << endl;
		}
		
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

		if (identical)
		{
			cout << "\t[Passed]: Topology match" << endl;
		}
		else
		{
			cerr << "\t[Failed]: result doesn't match with reference, if area match, consider update test references" << endl;
		}
	}
}

void generateReference(ifstream& listFs)
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

void printUsage()
{
	cout << u8"usage : (1) convexDecompCli.exe genRef <testList>" << endl;
	cout << u8"usage : (2) convexDecompCli.exe runTests <testList>" << endl;
}

int main(int narg, char** args)
{
    string listFile;
	if (narg != 3)
	{
		printUsage();
		return 1;
	}

	listFile = args[narg - 1];
	string cmd(args[narg - 2]);
	bool runTest = false;

	ifstream listFs(listFile);
	if (!listFs.is_open())
	{
		cerr << "Can't open test list file" << endl;
		return 1;
	}

	if(cmd == "runTests")
	{ 
		compareWithRef(listFs);
	}
	else if(cmd == "genRef")
	{
		generateReference(listFs);
	}
	else
	{
		cerr << "unknown command" << endl;
		printUsage();
		return 1;
	}
    return 0;
}

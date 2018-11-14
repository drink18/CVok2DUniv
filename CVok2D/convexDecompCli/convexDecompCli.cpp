#include <acd.h>
#include <iostream>
#include <string>
#include <fstream>

using namespace acd;
using namespace std;
int main(int narg, char** args)
{
    if(narg != 2)
    {
        cout << "usage: convexDecompCli graph_file_list.txt" << endl;
        return 1;
    }

    string listFile(args[1]);

    ifstream listFs(listFile);
    string testFileName;
    while(listFs >> testFileName)
    {
		string oname = string("out_" + testFileName);
		ifstream testFs(testFileName);
		vector<Polygon> poly = acd::readPolygon(testFs);
		vector<Polygon> doneList;
		for (auto& p : poly)
		{
			p.initializeAll();
			auto res = acd::_resolveLoop_All(p);
			for (int ip = 0; ip < res.size(); ++ip)
			{
				doneList.push_back(res[ip]);
			}
		}

		ofstream ofs(oname, ios::out);
		writePolygonListInfo(ofs, doneList);
    }

    return 0;
}

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
		ifstream testFs(testFileName);
		Polygon poly = acd::readPolygon(testFs);

    }


    return 0;
}

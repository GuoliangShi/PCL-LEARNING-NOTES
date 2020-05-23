#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include<iostream>
#include<string>

using namespace std;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

void ply2pcd(const string &name1, const string &name2)
{
	pcl::PCLPointCloud2 cloud;
	pcl::PLYReader reader;
	reader.read(name1, cloud);
	pcl::PCDWriter writer;
	writer.writeASCII(name2, cloud);
}

void pcd2ply(const string &name1, const string &name2)
{
	pcl::PCLPointCloud2 cloud;
	pcl::io::loadPCDFile(name1, cloud);
	pcl::PLYWriter Writer;
	Writer.write(name2, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), true, true);
}

int main()
{
	cout << "PCD TO PLY, ENTER 1: \n";
	cout << "PLY TO PCD, ENTER 2: \n";
	int function = 0;
	while (cin >> function&&function != 1 && function != 2) {
		cout << "MUST CHOOSE 1 OR 2!\n";
	}

	cin.clear();
	while (cin.get() != '\n')
		continue;

	cout << "INPUT FILE NAME: ";
	string input_name;
	getline(cin, input_name);
	cout << "OUTPUT FILE NAME: ";
	string output_name;
	getline(cin, output_name);

	if (function == 1)
		pcd2ply(input_name, output_name);
	else if (function == 2)
		ply2pcd(input_name, output_name);
	return 0;
}
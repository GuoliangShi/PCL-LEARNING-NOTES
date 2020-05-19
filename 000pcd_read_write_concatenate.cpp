#include<iostream>
#include<pcl/io/pcd_io.h>
#include<pcl/point_types.h>

template<typename T>
void show(T &cloud)
{
	for (size_t i = 0; i < cloud.points.size(); i++) {
		std::cout << cloud.points[i].x << " " <<
			cloud.points[i].y << " " <<
			cloud.points[i].z << std::endl;
	}
	std::cout << std::endl;
}

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ> cloud1;
	pcl::PointCloud<pcl::PointXYZ> cloud2;
	pcl::PointCloud<pcl::PointNormal> cloud3;
	if(pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\13427\\Desktop\\test.pcd",*cloud)==-1){
		PCL_ERROR("couldn't find file!");
		return -1;
	}
	cloud1.height = 1;
	cloud1.width = 5;
	cloud1.resize(cloud1.height*cloud1.width);
	cloud1.is_dense = false;
	for (size_t i = 0; i < cloud1.points.size(); i++) {
		cloud1.points[i].x = rand() / (RAND_MAX + 1.0f);
		cloud1.points[i].y = rand() / (RAND_MAX + 1.0f);
		cloud1.points[i].z = rand() / (RAND_MAX + 1.0f);
	}
	std::cout << "cloud data: " << std::endl;
	show(*cloud);
	std::cout << "cloud1 data: " << std::endl;
	show(cloud1);

	//Merge two point clouds
	std::cout << "cloud + cloud1 data: " << std::endl;
	cloud2 = *cloud + cloud1;
	show(cloud2);
	pcl::io::savePCDFileASCII("C:\\Users\\13427\\Desktop\\test_concated.pcd", cloud2);

	//concated two point clouds character
	std::cout << "cloud + cloud1 data: " << std::endl;
	pcl::concatenateFields(*cloud, cloud1, cloud3);
	show(cloud3);
	pcl::io::savePCDFileASCII("C:\\Users\\13427\\Desktop\\test_concated2.pcd", cloud3);
}
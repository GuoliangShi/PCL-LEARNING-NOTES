#include<iostream>
#include<pcl/point_cloud.h>
#include<pcl/io/io.h>
#include<pcl/point_types.h>
#include<pcl/octree/octree_search.h>
#include <pcl/octree/octree_pointcloud_changedetector.h>
#include<vector>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->height = 1000;
	cloud->width = 1;
	cloud->is_dense = true;
	cloud->resize(cloud->width*cloud->height);
	for (size_t i = 0; i < cloud->size(); i++) {
		cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}
	
	float resolution = 200.0f;
	pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud < pcl::PointXYZ>);
	cloud1->height = 1000;
	cloud1->width = 1;
	cloud1->is_dense = true;
	cloud1->resize(cloud1->height*cloud1->width);
	for (size_t j = 0; j < cloud1->size(); j++) {
		cloud1->points[j].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud1->points[j].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud1->points[j].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}

	octree.switchBuffers();
	octree.setInputCloud(cloud1);
	octree.addPointsFromInputCloud();

	vector<int> Idx;
	octree.getPointIndicesFromNewVoxels(Idx);

	cout << "There are " << Idx.size() << " new voxels" << endl;;
	for (size_t k = 0; k < Idx.size(); k++) {
		cout << "#" << k << ": " << cloud1->points[Idx[k]].x << " "
			<< cloud1->points[Idx[k]].y << " "
			<< cloud1->points[Idx[k]].z << endl;
	}
}
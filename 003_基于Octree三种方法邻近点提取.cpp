#include<iostream>
#include<pcl/point_cloud.h>
#include<pcl/io/io.h>
#include<pcl/point_types.h>
#include<pcl/octree/octree_search.h>
#include<vector>

using namespace std;

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->height = 100000;
	cloud->width = 1;
	cloud->is_dense = true;
	cloud->resize(cloud->width*cloud->height);
	for (size_t i = 0; i < cloud->size(); i++) {
		cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}
	// construct octree
	float resolution = 128.0f;
	pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
	octree.setInputCloud(cloud);
	octree.addPointsFromInputCloud();
	// set search point
	pcl::PointXYZ searchPoint;
	searchPoint = cloud->points[10];

	// neighbors within voxel search
	vector<int> Idx1;
	octree.voxelSearch(searchPoint, Idx1);
	cout << "Neighbors within voxel search points:\n";
	for (size_t i = 0; i < 10; i++) {
		cout << cloud->points[Idx1[i]].x << " "
			 << cloud->points[Idx1[i]].y << " "
			 << cloud->points[Idx1[i]].z << endl;
	}

	// K nearest Neighbor search
	int K = Idx1.size();
	vector<int> Idx2;
	vector<float> Distance1;
	octree.nearestKSearch(searchPoint, K, Idx2, Distance1);
	cout << "\nK nearest Neighbor search points:\n";
	for (size_t i = 0; i < 10; i++) {
		cout << cloud->points[Idx2[i]].x << " "
			 << cloud->points[Idx2[i]].y << " "
			 << cloud->points[Idx2[i]].z << endl;
	}

	// Neighbors within radius search
	float radius = resolution;
	vector<int> Idx3;
	vector<float> Distance2;
	octree.radiusSearch(searchPoint, radius, Idx3, Distance2);
	cout << "\nNeighbors within radius search points:\n";
	for (size_t i = 0; i < 10; i++) {
		cout << cloud->points[Idx3[i]].x << " "
			<< cloud->points[Idx3[i]].y << " "
			<< cloud->points[Idx3[i]].z << endl;
	}
}

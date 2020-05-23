#include<pcl/point_cloud.h>
#include<pcl/kdtree/kdtree_flann.h>

#include<vector>
#include<iostream>
#include<ctime>

int main()
{
	srand(time(NULL));
	time_t begin, end;
	begin = clock();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	cloud->height = 1;
	cloud->width = 100000;
	cloud->is_dense = true;
	cloud->resize(cloud->width*cloud->height);

	for (int i = 0; i < cloud->size(); i++) {
		cloud->points[i].x = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].y = 1024.0f*rand() / (RAND_MAX + 1.0f);
		cloud->points[i].z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	}

	// creats kdtree object
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//set input point cloud
	kdtree.setInputCloud(cloud);
	// set target point
	pcl::PointXYZ target;
	target.x = 1024.0f*rand() / (RAND_MAX + 1.0f);
	target.y = 1024.0f*rand() / (RAND_MAX + 1.0f);
	target.z = 1024.0f*rand() / (RAND_MAX + 1.0f);
	// set kdtree search 
	int K = 10;
	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	std::cout << "K nearset neighbor search at (" <<
		target.x << " " << target.y << " " << target.z <<
		"), with K = " << K << std::endl;

	if (kdtree.nearestKSearch(target, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
		for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
			std::cout << "    " << cloud->points[pointIdxNKNSearch[i]].x
			<< " " << cloud->points[pointIdxNKNSearch[i]].y
			<< " " << cloud->points[pointIdxNKNSearch[i]].z
			<< " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
	}

	end = clock();
	double time = (end - begin) * 1000 / CLOCKS_PER_SEC;
	std::cout << "time: " << time << "ms" << std::endl;

	return 0;
}
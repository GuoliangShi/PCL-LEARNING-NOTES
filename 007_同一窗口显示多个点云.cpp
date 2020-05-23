#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud4(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::io::loadPCDFile("C:\\Users\\13427\\Desktop\\maize.pcd", *cloud1);
	pcl::io::loadPCDFile("C:\\Users\\13427\\Desktop\\room_scan1.pcd", *cloud2);
	pcl::io::loadPCDFile("C:\\Users\\13427\\Desktop\\maize.pcd", *cloud3);
	pcl::io::loadPCDFile("C:\\Users\\13427\\Desktop\\room_scan1.pcd", *cloud4);

	pcl::PointCloud<pcl::PointXYZ>::Ptr data[] = { cloud1,cloud2,cloud3,cloud4 };

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	viewer->addPointCloud<pcl::PointXYZ>(cloud1, "sample cloud");
	//viewer->addPointCloud<pcl::PointXYZ>(cloud2, "sample cloud1");
	//viewer->removePointCloud("sample cloud1");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();

	int i = 1;
	while (!viewer->wasStopped()) {
		viewer->updatePointCloud(data[i], "sample cloud");
		i = (i + 1) % 4;
		viewer->spinOnce(1000);
	}
	return 0;
}

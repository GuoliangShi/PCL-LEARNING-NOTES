/* \author Geoffrey Biggs */

#include <iostream>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
// 帮助
void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-s           PointCloudColorHandlerRGBField example\n"
		<< "-g           PointCloudColorHandlerGenericField example\n"
		<< "-c           PointCloudColorHandlerCustom example\n"
		<< "-r           PointCloudColorHandlerRandom example\n"
		<< "-n           Normal visulization example\n"
		<< "\n\n";
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> colorHandler(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Cloud"));
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> genericHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Cloud"));
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(cloud, "y");
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> customHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> rgb(cloud, 0, 255, 255);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> randomHandler(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(cloud);
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	return viewer;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> normalHandler
						(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr cloud1)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer);
	pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ> rgb(cloud, "z");
	viewer->addPointCloud(cloud, rgb, "sample cloud");
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cloud, cloud1, 10, 0.05, "normals");
	return viewer;
}

// -----Main-----
int
main(int argc, char** argv)
{
	// 解析命令行参数

	printUsage(argv[0]);

	std::cout << "Input your commend: ";
	std::string commend;
	getline(cin, commend);

	bool simple(false), rgb(false), custom_c(false), normals(false),shapes(false);
	if (commend == "-s")
	{
		simple = true;
	}
	else if (commend == "-g")
	{
		rgb = true;
	}
	else if (commend == "-c")
	{
		custom_c = true;
	}
	else if (commend == "-r")
	{
		normals = true;
	}
	else if (commend == "-n")
	{
		shapes = true;
	}

	// 自行创建一随机点云
	pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
	std::cout << "Genarating example point clouds.\n\n";
	// 以椭圆为边线沿z轴拉伸获取其点云，并赋予红绿蓝渐变色。
	uint8_t r(255), g(15), b(15);
	for (float z(-1.0); z <= 1.0; z += 0.05)
	{
		for (float angle(0.0); angle <= 360.0; angle += 5.0)
		{
			pcl::PointXYZ basic_point;
			basic_point.x = 0.5 * cosf(pcl::deg2rad(angle));
			basic_point.y = sinf(pcl::deg2rad(angle));
			basic_point.z = z;
			basic_cloud_ptr->points.push_back(basic_point);

			pcl::PointXYZRGB point;
			point.x = basic_point.x;
			point.y = basic_point.y;
			point.z = basic_point.z;
			uint32_t rgb = (static_cast<uint32_t>(r) << 16 |
				static_cast<uint32_t>(g) << 8 | static_cast<uint32_t>(b));
			point.rgb = *reinterpret_cast<float*>(&rgb);
			point_cloud_ptr->points.push_back(point);
		}
		if (z < 0.0)
		{
			r -= 12;
			g += 12;
		}
		else
		{
			g -= 12;
			b += 12;
		}
	}
	basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
	basic_cloud_ptr->height = 1;
	point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
	point_cloud_ptr->height = 1;
	// 0.05为搜索半径获取点云法线
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
	ne.setInputCloud(point_cloud_ptr);
	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
	ne.setSearchMethod(tree);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.05);
	ne.compute(*cloud_normals1);
	//  0.1为搜索半径获取点云法线
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
	ne.setRadiusSearch(0.1);
	ne.compute(*cloud_normals2);

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
	if (simple)
	{
		viewer = colorHandler(point_cloud_ptr);
	}
	else if (rgb)
	{
		viewer = genericHandler(basic_cloud_ptr);
	}
	else if (custom_c)
	{
		viewer = customHandler(basic_cloud_ptr);
	}
	else if (normals)
	{
		viewer = randomHandler(basic_cloud_ptr);
	}
	else if (shapes)
	{
		viewer = normalHandler(basic_cloud_ptr, cloud_normals1);
	}

	viewer->setBackgroundColor(1, 1, 1);
	viewer->addCoordinateSystem(1.0);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->initCameraParameters();
	
	// 主循环
	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}



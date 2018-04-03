#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/common/common.h>

int main(int argc, char** argv)
{
 //retrieve arguments
char* pcdFile = argv[1];

std::cout << argv[1] << std::endl;
// initialize PointClouds
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

// Fill in the cloud data
pcl::PCDReader reader;
// Replace the path below with the path where you saved your file
reader.read (pcdFile, *cloud); // Remember to download the file first!

pcl::PointXYZRGBA mins;
pcl::PointXYZRGBA maxs;

pcl::getMinMax3D(*cloud, mins, maxs);
float height = maxs.y - mins.y;
float width = maxs.x - mins.x;

printf("height: %f, width: %f\n", height, width);

}

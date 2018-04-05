#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include "random_sample_consensus.h"
#include "voxel_grid.h"
#include "load_pcd.h"
#include "plane_seg.h"
#include "cluster_extraction.h"
#include "passthrough.h"
#include "shape_detection.h"

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

loadPcd(argc, pcdFile, cloud);

passthroughfilter(cloud, cloud);
loadPcd(argc, pcdFile, cloud);

std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
     << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

voxelfilter(cloud, cloud);
loadPcd(argc, pcdFile, cloud);

ransac(cloud, cloud,argv, argc);
planeSeg(cloud, cloud);
loadPcd(argc, pcdFile, cloud);

clusterExtraction(cloud, cloud);

}

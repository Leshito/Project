#include <iostream>
#include <Eigen/Geometry>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
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
#include "correspondence_grouping.h"
#include <vector>
#include "outputPoints.h"
//#include "load_arm_clusters.h"

int main(int argc, char** argv)
{
  //path to directory of the arm_clusters
  const char *dir_path="/home/saul/Documents/Project/pcdFiles/hand_clusters/";
  //num of files  in the arm_cluster directory
  int numOfFiles=4;
  std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > arm_clusters;
  //std::vector <pcl::PointXYZ> data
  std::vector < pcl::PointCloud<pcl::PointXYZRGBA>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZRGBA>::Ptr > > clusters;

  //load arm clusters
  //Set up file variables
  //std::stringstream ss;
  const char *pattern   = "arm_cluster_";
  const char *extension = ".pcd";


  for (int i=0; i<numOfFiles; i++)
  {
    std::stringstream ss;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tcloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
    //the dir_path is assumed to have the "/" symbol at the end of the string
    ss << dir_path << pattern << i << extension;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>(ss.str(), *tcloud);
    arm_clusters.push_back(tcloud);

  }

  //retrieve arguments
 char* pcdFile = argv[1];

 /*
  for (int i=0; i<numOfFiles; i++)
  {
     loadPcd(argc, pcdFile, arm_clusters[i]);
  }
*/



std::cout << argv[1] << std::endl;
// initialize PointClouds
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

// Fill in the cloud data
pcl::PCDReader reader;
// Replace the path below with the path where you saved your file
reader.read(pcdFile, *cloud); // Remember to download the file first!

loadPcd(argc, pcdFile, cloud);

passthroughfilter(cloud, cloud);
//loadPcd(argc, pcdFile, cloud);

//std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
//     << " data points (" << pcl::getFieldsList (*cloud) << ")." << std::endl;

voxelfilter(cloud, cloud);
//loadPcd(argc, pcdFile, cloud);


float planeCoe[] = {0, 0, 0, 0};

planeSeg(cloud, cloud, planeCoe);
loadPcd(argc, pcdFile, cloud);

clusterExtraction(cloud, &clusters);

int s= clusters.size();
//printf("test1: clusters.size() %d \n", s);
//find model
int flag=0;
for(int j=0; j<s;j++)
{
  //printf("j: %d\n", j);
  for(int i=0; i<numOfFiles;i++)
  {
    //printf("i: %d\n", i);
    if(findModel(argc,argv, arm_clusters[i], clusters[j])==1)
    {
      //rintf("match!\n");
      clusters.erase(clusters.begin()+j);
      flag=1;
      break;
    }
  }
  if(flag==1)
    break;
}

  //printf("final product\n");
  loadPcd(argc, pcdFile, clusters[0]);
  shapeDetect(clusters[0]);


  float heightWidth[] = {0, 0};
  getDimensions(planeCoe, clusters[0], heightWidth);
  printf("height: %f, width: %f\n", 39.37007874*heightWidth[0], 39.37007874*heightWidth[1]);
}

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

bool openCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut, const char* fileName)
{
    // convert the file name to string
    std::string fileNameStr(fileName);

    // handle various file types
    std::string fileExtension = fileNameStr.substr(fileNameStr.find_last_of(".") + 1);
    if(fileExtension.compare("pcd") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPCDFile<pcl::PointXYZRGBA>(fileNameStr, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcd file: %s \n", fileNameStr.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else if(fileExtension.compare("ply") == 0)
    {
        // attempt to open the file
        if(pcl::io::loadPLYFile<pcl::PointXYZRGBA>(fileNameStr, *cloudOut) == -1)
        {
            PCL_ERROR("error while attempting to read pcl file: %s \n", fileNameStr.c_str());
            return false;
        }
        else
        {
            return true;
        }
    }
    else
    {
        PCL_ERROR("error while attempting to read unsupported file: %s \n", fileNameStr.c_str());
        return false;
    }
}


int main(int argc, char** argv)
{
 //retrieve arguments
char* pcdFile = argv[2];
// initialize PointClouds
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr final (new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr voxCloud (new pcl::PointCloud<pcl::PointXYZRGBA>);

// Fill in the cloud data
pcl::PCDReader reader;
// Replace the path below with the path where you saved your file
reader.read (pcdFile, *cloud); // Remember to download the file first!

openCloud(cloud, pcdFile);

std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
     << " data points (" << pcl::getFieldsList (*cloud) << ").";

voxelfilter(cloud,voxCloud);
ransac(voxCloud, final,argv, argc);

}

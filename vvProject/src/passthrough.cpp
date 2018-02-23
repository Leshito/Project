
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

int passthroughfilter (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered)
{

  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGBA> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  pass.filter (*cloud_filtered);

  return (0);
}


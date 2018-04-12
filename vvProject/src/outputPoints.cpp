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
#include <pcl/common/common.h>

void getDimensions(
 	float plane[],
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud,
	float heightWidth[])
{
	pcl::PointXYZRGBA farthest;
	pcl::PointXYZRGBA nearest;
	float max = 0, min = 0;
	float a = plane[0];
	float b = plane[1];
	float c = plane[2];
	float d = plane[3];
	//Normalize plane normal


	//if (cloud->height == 1)
	//{
	//	std::cout << "unorganized" << std::endl;
	//}
	//else
	//{
		for(const pcl::PointXYZRGBA pt : cloud->points)
		{
			//printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
			if(!(std::isnan(pt.x)))
			{
				float dist = std::abs(a*pt.x + b*pt.y + c*pt.z + d)
							 /std::sqrt(std::pow(a, 2) + std::pow(b, 2) + std::pow(c, 2));
				//printf("%f\n", dist);
				if (dist < min)
				{
					min = dist;
					nearest = pt;
				}
				else if (dist > max)
				{
					max = dist;
					farthest = pt;
				}


				//printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
			}		
		}


	heightWidth[0] = max - min;

	if(min - 0.01 <= 0)
	{
		heightWidth[0] += 0.01;
	}

	pcl::PointXYZRGBA mins;
	pcl::PointXYZRGBA maxs;

	pcl::getMinMax3D(*cloud, mins, maxs);
	//float height = maxs.y - mins.y;
	 heightWidth[1] = maxs.x - mins.x;

	//}
}

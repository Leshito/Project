#include <pcl/io/pcd_io.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <iostream>
#include "shape_detection.h"

void shapeDetect(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud)
{
	// Get the model, if present.
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::SACSegmentation<pcl::PointXYZRGBA> segmentation;
	segmentation.setInputCloud(cloud);
	//segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.setMethodType(pcl::SAC_RANSAC);
	segmentation.setDistanceThreshold(0.01);
	segmentation.setOptimizeCoefficients(true);
	pcl::PointIndices::Ptr shapeIndices(new pcl::PointIndices);
	
	//Cylinder model
	segmentation.setModelType(pcl::SACMODEL_CYLINDER);
	segmentation.segment(*shapeIndices, *coefficients);
	if (shapeIndices->indices.size() != 0){
		std::cout << "Cylinder" << std::endl;
		return;
	}

	//Sphere model
	segmentation.setModelType(pcl::SACMODEL_SPHERE);
	segmentation.segment(*shapeIndices, *coefficients);
	if (shapeIndices->indices.size() != 0){
		std::cout << "Sphere" << std::endl;
		return;
	}

	/*//Plane model
	segmentation.setModelType(pcl::SACMODEL_PLANE);
	segmentation.segment(*shapeIndices, *coefficients);
	if (shapeIndices->indices.size() != 0){
		std::cout << "Plane in the scene." << std::endl;
		return;
	}*/
	std::cout << "Undetermined" << std::endl;
	

	//pcl::PCDWriter writer;
	//writer.write ("/home/luish/Schoolz/SD/Project/pcdFiles/objectsOnTable.pcd", *objects, 
    // false);
}

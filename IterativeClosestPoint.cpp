#include "IterativeClosestPoint.h"


IterativeClosestPoint::IterativeClosestPoint()
{
	_original_init = false;
}


IterativeClosestPoint::~IterativeClosestPoint()
{
}

void IterativeClosestPoint::initializeParameters() {
	//Init ICP parameters with params
	
}
bool IterativeClosestPoint::compute(CloudType cloud1, CloudType cloud2, float &roll, float &pitch, float &yaw) {

	CloudType::Ptr c1(new CloudType(cloud1));
	CloudType::Ptr c2(new CloudType(cloud2));
	//Prepare arguments
	icp.setInputCloud(c1);
	icp.setInputTarget(c2);
	pcl::PointCloud<pcl::PointXYZ> Final;
	//Align
	icp.align(Final);
	Eigen::Matrix4f transformation = icp.getFinalTransformation();
	
	
	//Get rotations
	roll = 0;
	pitch = 0;
	yaw = 0;
	return true;
}
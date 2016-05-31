#pragma once
#include "TrackingAlgorithm.h"



class IterativeClosestPoint :
	public TrackingAlgorithm
{
public:
	IterativeClosestPoint();
	~IterativeClosestPoint();
	bool compute(pcl::PointCloud<pcl::PointXYZ> * cloud1, pcl::PointCloud<pcl::PointXYZ> * cloud2, float &roll, float &pitch, float &yaw);
	void initializeParameters();

private:
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

};


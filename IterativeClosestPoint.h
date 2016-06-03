#pragma once
#include "TrackingAlgorithm.h"



class IterativeClosestPoint :
	public TrackingAlgorithm
{
public:
	IterativeClosestPoint();
	~IterativeClosestPoint();
	bool compute(CloudType cloud1, CloudType cloud2, float &roll, float &pitch, float &yaw);
	void initializeParameters();

private:
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

	bool _original_init;
};


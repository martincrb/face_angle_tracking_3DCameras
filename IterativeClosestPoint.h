#pragma once
#include "TrackingAlgorithm.h"


#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_estimation_organized_projection.h>

#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_median_distance.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>
#include <pcl/registration/correspondence_rejection_organized_boundary.h>

#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_svd.h>
class IterativeClosestPoint :
	public TrackingAlgorithm
{
public:
	IterativeClosestPoint();
	~IterativeClosestPoint();
	bool compute(CloudType cloud1, CloudType cloud2, float &roll, float &pitch, float &yaw, double &fitness);
	void initializeParameters();

private:
	//pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icpN;
	//Rejectors
	pcl::registration::CorrespondenceRejectorDistance::Ptr _rejector_distance;
	pcl::registration::CorrespondenceRejectorMedianDistance::Ptr _rejector_median_distance;
	pcl::registration::CorrespondenceRejectorOneToOne::Ptr _rejector_one_to_one;
	pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr _rejector_normal;
	pcl::registration::CorrespondenceRejectionOrganizedBoundary::Ptr _rejector_boundary;

	//Errors
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr _error_svd;
	pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>::Ptr _error_point_plane;

	//Correspondence estimators
	pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr estProj; //Projection correspondence
	pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointNormal, pcl::PointNormal, float>::Ptr estProjN; //Projection correspondence
	pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr estSVD;
	pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal, float>::Ptr estSVDN;
	bool _original_init;
	
	bool _normals;

	bool _error_p_plane;
	bool _error_p_svd;
	
	bool _p_point;
	bool _rej_dist;
	bool _rej_median;
	bool _rej_onetone;
	bool _rej_normal;
	bool _rej_bound;

	double _rej_dist_threshold, _rej_median_threshold, _rej_normal_threshold;
	Eigen::Matrix4f _last_transform;

	float _epsilon;
	float _transform_epsilon;
	int _max_iter;
	float _max_corresp_dist;
};


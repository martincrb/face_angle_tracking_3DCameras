#include "IterativeClosestPoint.h"


IterativeClosestPoint::IterativeClosestPoint()
{
	_original_init = false;
	_last_transform << 1, 0, 0, 0,
						0, 1, 0, 0,
						0, 0, 1, 0,
						0, 0, 0, 1;

	//Init rejectors
	_rejector_distance = pcl::registration::CorrespondenceRejectorDistance::Ptr(new pcl::registration::CorrespondenceRejectorDistance);
	_rejector_median_distance = pcl::registration::CorrespondenceRejectorMedianDistance::Ptr(new pcl::registration::CorrespondenceRejectorMedianDistance);
	_rejector_one_to_one = pcl::registration::CorrespondenceRejectorOneToOne::Ptr(new pcl::registration::CorrespondenceRejectorOneToOne);
	_rejector_normal = pcl::registration::CorrespondenceRejectorSurfaceNormal::Ptr(new  pcl::registration::CorrespondenceRejectorSurfaceNormal);
	_rejector_boundary = pcl::registration::CorrespondenceRejectionOrganizedBoundary::Ptr(new pcl::registration::CorrespondenceRejectionOrganizedBoundary);
	
	//init errors
	_error_svd = pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Ptr(new pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>);
	_error_point_plane = pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>::Ptr(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal>);
	
	//init estimators
	estProj = pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr(new  pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointXYZ, pcl::PointXYZ, float>);
	estProjN = pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointNormal, pcl::PointNormal, float>::Ptr(new  pcl::registration::CorrespondenceEstimationOrganizedProjection<pcl::PointNormal, pcl::PointNormal, float>);
	estSVD = pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, float>::Ptr(new pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ, float>);
	estSVDN = pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal, float>::Ptr(new pcl::registration::CorrespondenceEstimation<pcl::PointNormal, pcl::PointNormal, float>);
}


IterativeClosestPoint::~IterativeClosestPoint()
{
}

void IterativeClosestPoint::initializeParameters() {
	//Init ICP parameters with params
	_epsilon = params["epsilon"].toFloat();
	_normals = params["normals"] == "true";
	_transform_epsilon = params["epsilon_transform"].toFloat();
	_max_iter = params["max_iter"].toInt();
	_max_corresp_dist = params["max_corresp_dist"].toInt();

	_rej_bound = params["boundary"] == "true";
	_rej_onetone = params["onetoone"] == "true";
	_rej_median = params["rej_median"] == "true";
	_rej_dist = params["rej_dist"] == "true";
	_rej_normal = params["rej_normals"] == "true";
	if (!_normals) {
		//Normal ICP
		if (params["correspondemce"] == "projection") {
			icp.setCorrespondenceEstimation(estProj);
			estProj->setFocalLengths(params["focal_length"].toFloat(), params["focal_length"].toFloat());
		}
		else if (params["correspondence"] == "nearest") {
			icp.setCorrespondenceEstimation(estSVD);
		}
		icp.setEuclideanFitnessEpsilon(_epsilon);
		icp.setTransformationEpsilon(_transform_epsilon);
		icp.setMaximumIterations(_max_iter);
		icp.setMaxCorrespondenceDistance(_max_corresp_dist);
		if (_rej_bound) {
			icp.addCorrespondenceRejector(_rejector_boundary);
		}
		if (_rej_onetone) {
			icp.addCorrespondenceRejector(_rejector_one_to_one);
		}
		if (_rej_median) {
			_rej_median_threshold = params["rej_median_threshold"].toFloat();
			_rejector_median_distance->setMedianFactor(_rej_median_threshold);
			icp.addCorrespondenceRejector(_rejector_median_distance);
		}
		if (_rej_dist) {
			_rej_dist_threshold = params["rej_dist_threshold"].toFloat();
			_rejector_distance->setMaximumDistance(_rej_dist_threshold);
			icp.addCorrespondenceRejector(_rejector_distance);
		}
		if (_rej_normal){
			_rej_normal_threshold = params["rej_normals_threshold"].toFloat();
			_rejector_normal->setThreshold(_rej_normal_threshold);
			icp.addCorrespondenceRejector(_rejector_normal);
		}
	}
	else {
		//Point to plane ICP
		if (params["correspondemce"] == "projection") {
			icpN.setCorrespondenceEstimation(estProjN);
		}
		else if (params["correspondence"] == "nearest") {
			icpN.setCorrespondenceEstimation(estSVDN);
		}
		icpN.setEuclideanFitnessEpsilon(_epsilon);
		icpN.setTransformationEpsilon(_transform_epsilon);
		icpN.setMaximumIterations(_max_iter);
		icpN.setMaxCorrespondenceDistance(_max_corresp_dist);
		icpN.setTransformationEstimation(_error_point_plane);
		if (_rej_bound) {
			icpN.addCorrespondenceRejector(_rejector_boundary);
		}
		if (_rej_onetone) {
			icpN.addCorrespondenceRejector(_rejector_one_to_one);
		}
		if (_rej_median) {
			_rej_median_threshold = params["rej_median_threshold"].toFloat();
			_rejector_median_distance->setMedianFactor(_rej_median_threshold);
			icpN.addCorrespondenceRejector(_rejector_median_distance);
		}
		if (_rej_dist) {
			_rej_dist_threshold = params["rej_dist_threshold"].toFloat();
			_rejector_distance->setMaximumDistance(_rej_dist_threshold);
			icpN.addCorrespondenceRejector(_rejector_distance);
		}
		if (_rej_normal){
			_rej_normal_threshold = params["rej_normals_threshold"].toFloat();
			_rejector_normal->setThreshold(_rej_normal_threshold);
			icpN.addCorrespondenceRejector(_rejector_normal);
		}

	}
	icpN.setUseReciprocalCorrespondences(true);
	icp.setUseReciprocalCorrespondences(true);
	
	
	//icp.setCorrespondenceEstimation(estProj);
	
}
bool IterativeClosestPoint::compute(CloudType cloud1, CloudType cloud2, float &roll, float &pitch, float &yaw, double &fitness) {

	//assert(cloud1.size() > 0 && cloud2.size() > 0);
	//assert(cloud1.isOrganized() && cloud2.isOrganized());
	
	
	CloudType::Ptr c1(new CloudType(cloud1));
	CloudType::Ptr c2(new CloudType(cloud2));

	// Output datasets
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud1_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud2_normals(new pcl::PointCloud<pcl::PointNormal>);
	if (_normals) {
		// Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::PointNormal> ne;
		ne.setInputCloud(c1);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);

		// Use all neighbors in a sphere of radius 3cm
		ne.setRadiusSearch(0.03);

		// Compute the features
		ne.compute(*cloud1_normals);
		ne.setInputCloud(c2);
		ne.compute(*cloud2_normals);

		//Prepare arguments
		icpN.setInputCloud(cloud1_normals);
		icpN.setInputTarget(cloud2_normals);
	}
	pcl::PointCloud<pcl::PointNormal> FinalNormal;
	pcl::PointCloud<pcl::PointXYZ> Final;

	
	bool conv;
	Eigen::Matrix4f transform;
	double error;
	if (_normals) {
		icpN.align(FinalNormal, _last_transform);
		conv = icpN.hasConverged();
		transform = icpN.getFinalTransformation();
		_last_transform = transform;
		error = icpN.getFitnessScore();
	}
	else {
		icp.setInputCloud(c1);
		icp.setInputTarget(c2);
		icp.align(Final, _last_transform);
		conv = icp.hasConverged();
		transform = icp.getFinalTransformation();
		_last_transform = transform;
		error = icp.getFitnessScore();
	}
	fitness = error;
	if (conv) {
	//Get rotations
		glm::mat4x4 glm_transform = glm::mat4x4(transform(0, 0), transform(0, 1), transform(0, 2), transform(0, 3),
												transform(1, 0), transform(1, 1), transform(1, 2), transform(1, 3),
												transform(2, 0), transform(2, 1), transform(2, 2), transform(2, 3),
												transform(3, 0), transform(3, 1), transform(3, 2), transform(3, 3));
		double a, b, c;
		a = transform(1, 1);
		b = transform(1, 2);
		c = transform(1, 3);
		glm::vec3 abc = glm::vec3(transform(0, 0), transform(0, 1), transform(0, 2));
		glm::vec3 efg = glm::vec3(transform(1, 0), transform(1, 1), transform(1, 2));
		glm::vec3 ijk = glm::vec3(transform(2, 0), transform(2, 1), transform(2, 2));
		glm::vec3 trans(transform(3, 0), transform(3, 1), transform(3, 2));
		double scaleX = glm::length(abc);
		double scaleY = glm::length(efg);
		double scaleZ = glm::length(ijk);

		glm::vec3 tempZ = glm::cross(abc, ijk);
		if (glm::dot(tempZ, ijk) < 0) {
			scaleX = -scaleX;
			a = -a;
			b = -b;
			c = -c;
		}
		glm::normalize(glm::vec3(a,b,c));
		glm::normalize(efg);
		glm::normalize(ijk);

		auto rotation = glm::quat_cast(glm_transform);
		auto euler = glm::eulerAngles(rotation);
		auto r = glm::roll(rotation);
		auto p = glm::pitch(rotation);
		auto y = glm::yaw(rotation);
		//auto r = glm::degrees(euler[0]);
		//auto p = glm::degrees(euler[1]);
		//auto y = glm::degrees(euler[2]);
		roll = r * 250;
		pitch = p * 250;
		yaw = y * 250;
	}
	return conv;
}
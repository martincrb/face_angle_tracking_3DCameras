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
	_epsilon = params["epsilon"].toFloat();
	icp.setEuclideanFitnessEpsilon(_epsilon);
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
	bool conv = icp.hasConverged();
	auto transform = icp.getFinalTransformation();
	auto error = icp.getFitnessScore();
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
		auto r = glm::roll(rotation);
		auto p = glm::pitch(rotation);
		auto y = glm::yaw(rotation);
		roll = r * 1000;
		pitch = p * 1000;
		yaw = y * 1000;
	}
	return conv;
}
#pragma once
#include <qstring.h>
#include <qmap.h>
#undef max
#undef min
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/keypoints/uniform_sampling.h>

typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
class TrackingAlgorithm
{
public:
	TrackingAlgorithm();
	~TrackingAlgorithm();
	virtual bool compute(CloudType cloud1, CloudType cloud2, float &roll, float &pitch, float &yaw) = 0;
	void setParameters(QMap<QString, QString> p);
	QMap<QString, QString> getParameters();
	virtual void initializeParameters() = 0;

private:
	
	QMap<QString, QString> params;
};


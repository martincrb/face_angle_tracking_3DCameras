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
#include "glm/geometric.hpp"
#include "glm/gtc/quaternion.hpp"
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>


typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
typedef pcl::PointCloud<pcl::PointNormal> CloudTypeNormal;
class TrackingAlgorithm
{
public:
	TrackingAlgorithm();
	~TrackingAlgorithm();
	virtual bool compute(CloudType cloud1, CloudType cloud2, float &roll, float &pitch, float &yaw, double &fitness) = 0;
	void setParameters(QMap<QString, QString> p);
	QMap<QString, QString> getParameters();
	QString getParameter(const QString key);
	virtual void initializeParameters() = 0;

protected:
	
	QMap<QString, QString> params;
};


#pragma once
#include <qstring.h>
#include <qmap.h>
#undef max
#undef min
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
class TrackingAlgorithm
{
public:
	TrackingAlgorithm();
	~TrackingAlgorithm();
	virtual bool compute(pcl::PointCloud<pcl::PointXYZ> * cloud1, pcl::PointCloud<pcl::PointXYZ> * cloud2, float &roll, float &pitch, float &yaw) = 0;
	void setParameters(QMap<QString, QString> p);
	QMap<QString, QString> getParameters();
	virtual void initializeParameters() = 0;

private:
	
	QMap<QString, QString> params;
};


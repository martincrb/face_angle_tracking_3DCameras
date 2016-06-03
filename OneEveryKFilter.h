#pragma once
#undef max
#undef min
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/filter.h>
class OneEveryKFilter
{
public:
	OneEveryKFilter();
	OneEveryKFilter(int k);
	~OneEveryKFilter();

	void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out);
private:
	int k;
};


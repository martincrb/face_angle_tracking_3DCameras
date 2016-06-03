#include "OneEveryKFilter.h"


OneEveryKFilter::OneEveryKFilter()
{
}


OneEveryKFilter::~OneEveryKFilter()
{
}

OneEveryKFilter::OneEveryKFilter(int k) {
	this->k = k;
}

void OneEveryKFilter::filter(pcl::PointCloud<pcl::PointXYZ>::Ptr &in, pcl::PointCloud<pcl::PointXYZ>::Ptr &out) {
	int total_points = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered;
	for (unsigned int i = 0; i < in->points.size(); i += k) {
		++total_points;
		//filtered->insert()
	}
}
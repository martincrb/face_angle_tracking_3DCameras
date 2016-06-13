#pragma once
#include "TrackingAlgorithm.h"
#include "GLWidget.h"
#include <qimage.h>
#include "quaternion.hpp"
#include <QFile>
#include <QTextStream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#define MAX_FACES  1
typedef pcl::PointCloud<pcl::PointXYZ> CloudType;


class FaceTrackingApp;
class Camera3D
{
public:
	Camera3D();
	~Camera3D();
	virtual void init(FaceTrackingApp *app) = 0;
	virtual void stop() = 0;
	virtual void getFrameImage(QImage &image) = 0;
	virtual void getDepthPoints() = 0;
	virtual void update() = 0;
	virtual bool track() = 0;
	void setRenderer(GLWidget *renderer);
	void setTracker(TrackingAlgorithm *tracker);
	void useFace(bool face);
	bool trackerIsSet();
protected:
	GLWidget *renderer;
	TrackingAlgorithm *tracker;
	bool _tracker_set;
	bool _use_face;
	float _roll, _pitch, _yaw;
	FaceTrackingApp *app;
	
	CloudType::Ptr actual_cloud;
	CloudType::Ptr last_cloud;
	CloudType::Ptr first_cloud;
	//Filters
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;

};


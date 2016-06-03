#pragma once
#include "Camera3D.h"
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "pxcprojection.h"
#include <assert.h> 
typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

class RealSense :
	public Camera3D
{
public:
	RealSense();
	~RealSense();
	void init(FaceTrackingApp *app);
	void stop();
	void getFrameImage(QImage &image);
	void getDepthPoints();
	bool track();
	void update();
private:
	PXCCapture::Device *_device;
	PXCSenseManager *pp;
	PXCFaceModule* faceAnalyzer;
	PXCFaceData* outputData;
	pxcStatus sts;

	CloudType::Ptr actual_cloud;
	CloudType::Ptr last_cloud;
	CloudType::Ptr first_cloud;
	//Filters
	pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;

	bool _last_init;
};


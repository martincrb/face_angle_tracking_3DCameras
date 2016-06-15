#pragma once
#include "Camera3D.h"
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "pxcprojection.h"
#include <assert.h> 
#include <boost/geometry/geometry.hpp>
#include <QTime>


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

	
};


#pragma once
#include "Camera3D.h"
#include "pxcsensemanager.h"
#include "pxcmetadata.h"


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
	PXCSenseManager *pp;
	PXCFaceModule* faceAnalyzer;
	PXCFaceData* outputData;
	pxcStatus sts;
};


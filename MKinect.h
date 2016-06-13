#pragma once
#include "Camera3D.h"
#include <Windows.h>
#include <Kinect.h>
#include <Kinect.Face.h>
#include <assert.h> 
#include <boost/geometry/geometry.hpp>
#include <QTime>

#define KinectColorWidth 1920
#define KinectColorHeight 1080

class MKinect :
	public Camera3D
{
public:
	MKinect();
	~MKinect();
	void init(FaceTrackingApp *app);
	void stop();
	void getFrameImage(QImage &image);
	void getDepthPoints();
	void update();
	bool track();

private:
	// Kinect variables
	IKinectSensor* sensor;         // Kinect sensor
	IMultiSourceFrameReader* reader;     // Kinect color data source
	ICoordinateMapper* mapper;

	CameraSpacePoint depth2xyz[KinectColorWidth*KinectColorHeight];    // Maps depth pixels to 3d coordinates
	// OpenGL Variables
	GLuint textureId;              // ID of the texture to contain Kinect RGB Data
	GLubyte data[KinectColorWidth*KinectColorHeight * 4];  // BGRA array containing the texture data

	IFaceFrameSource* pFaceSource[BODY_COUNT];
	IFaceFrameReader* pFaceReader[BODY_COUNT];
	IBodyFrameSource* pBodySource;

	IBodyFrameReader* pBodyReader;
	int _actual_frame;
	void getKinectData(QImage &image);
	void getDepthData(IMultiSourceFrame* frame);
	void getColorData(IMultiSourceFrame* frame, QImage& dest);
};


#include "RealSense.h"
#include "facetrackingapp.h"
#include <iostream>

RealSense::RealSense()
{
	_tracker_set = false;
}


RealSense::~RealSense()
{
	stop();
}

void RealSense::init(FaceTrackingApp *app)
{
	this->app = app;
	pp = PXCSenseManager::CreateInstance();
	if (!pp) {
		std::cout << "Unable to create the SenseManager" << std::endl;
	}
	pp->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480);


	if (pp->EnableFace() < PXC_STATUS_NO_ERROR) {
		return;
	}
	faceAnalyzer = pp->QueryFace();
	if (!faceAnalyzer) {
		return;
	}
	outputData = faceAnalyzer->CreateOutput();
	if (!pp) {
		return;
	}
	if(pp->Init() < PXC_STATUS_NO_ERROR) return;


	PXCFaceConfiguration* config = faceAnalyzer->CreateActiveConfiguration();
	config->SetTrackingMode(PXCFaceConfiguration::TrackingModeType::FACE_MODE_COLOR_PLUS_DEPTH);
	config->detection.isEnabled = true;
	config->detection.maxTrackedFaces = MAX_FACES;
	config->ApplyChanges();

	

	QImage actualFrame;
	getFrameImage(actualFrame);
	renderer->initTexture(actualFrame);

	
}

void RealSense::update() {
	//if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
	//	return;

	QImage actualFrame;
	getFrameImage(actualFrame);
	renderer->initTexture(actualFrame);

	bool orientationIsValid = false;
	if (_tracker_set == false) { //Use built-in algorithm
		orientationIsValid = track();
	}
	else {
		//Get Points detected by camera

		//Then, filter points using filter object

		//Pass cloud to algorithm
		orientationIsValid = tracker->compute(nullptr, nullptr, _roll, _pitch, _yaw);
	}

	//pp->ReleaseFrame();

	renderer->setFaceTracked(orientationIsValid);
	app->setFaceTracked(orientationIsValid);
	if (orientationIsValid) {
		app->setFaceAngles(_yaw, _pitch, _roll);
	}
}

void RealSense::stop() {
	pp->Release();
}

void RealSense::getFrameImage(QImage &image)
{
	if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
		return;
	// retrieve the sample
	PXCCapture::Sample *sample = pp->QuerySample();

	//image is a PXCImage instance
	PXCImage::ImageData colorData, depthData;
	sample->color->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &colorData);
	int height = sample->color->QueryInfo().height;
	int width = sample->color->QueryInfo().width;

	// image planes are in data.planes[0-3] with pitch data.pitches[0-3]
	QImage colorImage(colorData.planes[0], width, height, QImage::Format_RGB32);
	image = colorImage;

	sample->color->ReleaseAccess(&colorData);
	pp->ReleaseFrame();

}
void RealSense::getDepthPoints()
{
}

bool RealSense::track() {
	if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
		return false;

	outputData->Update();
	bool valid = false;
	/* Detection Structs */
	PXCFaceData::DetectionData *detectionData;
	PXCRectI32 rectangle;


	// iterate through faces
	pxcU16 numOfFaces = outputData->QueryNumberOfDetectedFaces();
	for (pxcU16 i = 0; i < numOfFaces; i++)
	{
		// get face data by index
		PXCFaceData::Face *trackedFace = outputData->QueryFaceByIndex(i);
		if (trackedFace != NULL)
		{
			detectionData = trackedFace->QueryDetection();
			detectionData->QueryBoundingRect(&rectangle);

			//Pose
			PXCFaceData::PoseData *poseData = NULL;
			poseData = trackedFace->QueryPose();
			if (poseData != NULL) {
				PXCFaceData::PoseQuaternion faceRotation;
				if (poseData->QueryPoseQuaternion(&faceRotation)) {
					PXCFaceData::PoseEulerAngles eulerAngles;
					poseData->QueryPoseAngles(&eulerAngles);

					_yaw = eulerAngles.yaw;
					_pitch = eulerAngles.pitch;
					_roll = eulerAngles.roll;
					valid = true;
				}
			}
		}
	}
	pp->ReleaseFrame();
	return valid;
}
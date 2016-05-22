#include "RealSense.h"
#include "facetrackingapp.h"
#include <iostream>

RealSense::RealSense()
{
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
	QImage actualFrame;
	getFrameImage(actualFrame);
	renderer->initTexture(actualFrame);
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

	outputData->Update();

	/* Detection Structs */
	PXCFaceData::DetectionData *detectionData;
	PXCRectI32 rectangle;
	
	// iterate through faces
	renderer->setFaceTracked(false);
	app->setFaceTracked(false);
	pxcU16 numOfFaces = outputData->QueryNumberOfDetectedFaces();
	for (pxcU16 i = 0; i < numOfFaces; i++)
	{
		// get face data by index
		PXCFaceData::Face *trackedFace = outputData->QueryFaceByIndex(i);
		if (trackedFace != NULL)
		{
			renderer->setFaceTracked(true);
			app->setFaceTracked(true);

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
					geom::Quaternion q(faceRotation.x, faceRotation.y, faceRotation.z, faceRotation.w);
					geom::Quaternion view(0.0, 0.0, -1.0, 0.0);
					geom::Quaternion result = q * view * q.get_conjugate();
					auto faceViewDir_x = static_cast<float>(result.x);
					auto faceViewDir_y = static_cast<float>(result.y);
					auto faceViewDir_z = static_cast<float>(result.z);

					app->setFaceAngles(eulerAngles.yaw, eulerAngles.pitch, eulerAngles.roll);
				}
			}
			else {
				renderer->setFaceTracked(false);
				app->setFaceTracked(false);
			}
		}
		else {
			renderer->setFaceTracked(false);
			app->setFaceTracked(false);
		}
	}
	// go fetching the next sample
	pp->ReleaseFrame();
}
void RealSense::getDepthPoints()
{
}
void RealSense::getFaceOrientation()
{
}
void RealSense::getFaceOrientation(TrackingAlgorithm *tA)
{
}
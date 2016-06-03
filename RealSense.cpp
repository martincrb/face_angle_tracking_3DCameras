#include "RealSense.h"
#include "facetrackingapp.h"
#include <iostream>

RealSense::RealSense()
{
	_tracker_set = false;
	_last_init = false;
	last_cloud = CloudType::Ptr(new CloudType);
	actual_cloud = CloudType::Ptr(new CloudType);
	first_cloud = CloudType::Ptr(new CloudType);
}


RealSense::~RealSense()
{
	stop();
}

void RealSense::init(FaceTrackingApp *app)
{
	_last_init = false;
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
	_device = pp->QueryCaptureManager()->QueryDevice();

	PXCFaceConfiguration* config = faceAnalyzer->CreateActiveConfiguration();
	config->SetTrackingMode(PXCFaceConfiguration::TrackingModeType::FACE_MODE_COLOR_PLUS_DEPTH);
	config->detection.isEnabled = true;
	config->detection.maxTrackedFaces = MAX_FACES;
	config->ApplyChanges();

	

	QImage actualFrame;
	getFrameImage(actualFrame);
	renderer->initTexture(actualFrame);

	
}

/* Helper function to convert a PXCPoint3DF32 point into a PCL point.
* Takes care of unit conversion (PXC point coordinates are in millimeters)
* and invalid points. */
template <typename T> inline void
convertPoint(const PXCPoint3DF32& src, T& tgt)
{
	static const float nan = std::numeric_limits<float>::quiet_NaN();
	if (src.z == 0)
	{
		tgt.x = tgt.y = tgt.z = nan;
		//tgt.x = tgt.y = tgt.z = 0;

	}
	else
	{
		tgt.x = -src.x / 1000.0;
		tgt.y = src.y / 1000.0;
		tgt.z = src.z / 1000.0;
	}
}

void RealSense::update() {
	//if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
	//	return;

	QImage actualFrame;
	getFrameImage(actualFrame);
	renderer->initTexture(actualFrame);
	int w = 640;
	int h = 480;
	int SIZE = w*h;
	bool orientationIsValid = false;

	//Get Points detected by camera. TODO: Move to own func
	PXCProjection* projection = _device->CreateProjection();
	PXCCapture::Sample sample;

	std::vector<PXCPoint3DF32> vertices(SIZE);
	CloudType::Ptr xyz_cloud(new CloudType(w, h));


	PXCImage::ImageData data;
	PXCCapture::Sample *sam = pp->QuerySample();
	sam->depth->AcquireAccess(PXCImage::ACCESS_READ, &data);
	projection->QueryVertices(sam->depth, vertices.data());
	sam->depth->ReleaseAccess(&data);
	projection->Release();

	for (int i = 0; i < SIZE; i++) {
		convertPoint(vertices[i], xyz_cloud->points[i]);
	}

	


	if (!_last_init) {
		_last_init = true;
		//pcl::copyPointCloud(*xyz_cloud, *last_cloud);
		last_cloud->is_dense = false;
		first_cloud->is_dense = false;
		xyz_cloud->is_dense = false;
		std::vector<int> indices2;
		CloudType::Ptr cloud_filtered(new CloudType);
		pcl::removeNaNFromPointCloud(*xyz_cloud, *cloud_filtered, indices2);
		//for (int i = 0; i < cloud_filtered->size(); i++) {
		//	assert(!std::isnan(cloud_filtered->points[i].x));
		//}
		pcl::PointCloud<int> indicesIN;
		uniform_sampling.setInputCloud(cloud_filtered);
		uniform_sampling.setRadiusSearch(0.05f);
		uniform_sampling.compute(indicesIN);
		pcl::copyPointCloud(*cloud_filtered, indicesIN.points, *first_cloud);
		pcl::copyPointCloud(*cloud_filtered, indicesIN.points, *last_cloud);
		//Save in first_cloud the first frame, filtered and prepared :D
	}

	
	//for (int i = 0; i < _cloud1->size(); i++) {
	//	assert(!std::isnan(_cloud1->points[i].x));
	//}
	if (_tracker_set == false) { //Use built-in algorithm
		orientationIsValid = track();
	}
	else {
		//Filter NANS for ICP
		xyz_cloud->is_dense = false;

		std::vector<int> indices;
		CloudType::Ptr _cloud1(new CloudType);
		pcl::removeNaNFromPointCloud(*xyz_cloud, *_cloud1, indices);

		//Filter points using filter object
		CloudType cloud_in_key;
		
		pcl::PointCloud<int> indicesOUT;
		uniform_sampling.setInputCloud(_cloud1);
		uniform_sampling.setRadiusSearch(0.05f);
		uniform_sampling.compute(indicesOUT);

		pcl::copyPointCloud(*_cloud1.get(), indicesOUT.points, *actual_cloud);

		//Pass cloud to renderer (Or pass the filtered one to see differences)
		renderer->setCurrentFramePCL(actual_cloud);


		//Pass cloud to algorithm
		if (_last_init) { //Compute if we have 2 frames (last_cloud is filled with the first frame)
			orientationIsValid = tracker->compute(*last_cloud.get(), *actual_cloud.get(), _roll, _pitch, _yaw);
		}
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
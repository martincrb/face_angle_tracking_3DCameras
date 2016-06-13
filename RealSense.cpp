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
		app->setMessage("Unable to create the SenseManager");
	}
	_actual_frame = 0;

	if (pp->EnableStream(PXCCapture::STREAM_TYPE_COLOR, 640, 480) < PXC_STATUS_NO_ERROR) {
		
		app->setMessage("Unable to create Stream");
		return;

	}
	if (pp->EnableFace() < PXC_STATUS_NO_ERROR) {
		app->setMessage("Unable to create Face Module");
		return;
	}
	faceAnalyzer = pp->QueryFace();
	if (!faceAnalyzer) {
		app->setMessage("Unable to create Face Analyzer");
		return;
	}
	outputData = faceAnalyzer->CreateOutput();
	if (!pp) {
		return;
	}
	if (pp->Init() < PXC_STATUS_NO_ERROR){
		app->setMessage("Unable to init the SenseManager");
		return;
	}
	_device = pp->QueryCaptureManager()->QueryDevice();
	_device->SetMirrorMode(_device->MIRROR_MODE_HORIZONTAL);
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
template <typename T> inline bool
convertPoint(const PXCPoint3DF32& src, T& tgt, const PXCPoint3DF32 rectangle[4], bool useFace)
{
	static const float nan = std::numeric_limits<float>::quiet_NaN();
	if (src.z == 0 || src.z > 500)
		//Point is invalid
	{
		tgt.x = tgt.y = tgt.z = nan;
		return false;
		//tgt.x = tgt.y = tgt.z = 0;

	}
	else {
		if (!useFace) {
			tgt.x = -src.x / 1000.0;
			tgt.y = src.y / 1000.0;
			tgt.z = src.z / 1000.0;
			return true;
		}
		else {
			if (pointInsideRectangle(rectangle, src)) { //Point valid: Check whether lies inside face bbox
					tgt.x = -src.x / 1000.0;
					tgt.y = src.y / 1000.0;
					tgt.z = src.z / 1000.0;
					return true;
			}
			else {
				tgt.x = tgt.y = tgt.z = nan;
				return false;
			}
		}
	}
}
float sign(PXCPoint3DF32 p1, PXCPoint3DF32 p2, PXCPoint3DF32 p3)
{
	return (p1.x - p3.x) * (p2.y - p3.y) - (p2.x - p3.x) * (p1.y - p3.y);
}
bool pointInsideRectangle(const PXCPoint3DF32 rectangle[4], const PXCPoint3DF32& src) {
	bool b1, b2, b3;
	PXCPoint3DF32 v1;
	PXCPoint3DF32 v2;
	PXCPoint3DF32 v3;
	v1 = rectangle[1];

	v2 = rectangle[2];

	v3 = rectangle[3];

	b1 = sign(src, v1, v2) < 0.0f;
	b2 = sign(src, v2, v3) < 0.0f;
	b3 = sign(src, v3, v1) < 0.0f;

	return ((b1 == b2) && (b2 == b3));
}



void RealSense::update() {
	//if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
	//	return;
	if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
		return;

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
			valid = true;
			detectionData = trackedFace->QueryDetection();
			detectionData->QueryBoundingRect(&rectangle);
		}
	}
	pp->ReleaseFrame();
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

	PXCPoint3DF32 points3D[4];
	PXCPoint3DF32 points[4];
	PXCPointF32 pointsDepth[4];
	if (_use_face) {
		//Rectangle points to array
		
		points[0].x = rectangle.x;
		points[0].y = rectangle.y;
		points[0].z = 0;

		points[1].x = rectangle.x + rectangle.w;
		points[1].y = rectangle.y;
		points[1].z = 0;

		points[2].x = rectangle.x + rectangle.w;
		points[2].y = rectangle.y - rectangle.h;
		points[2].z = 0;

		points[3].x = rectangle.x;
		points[3].y = rectangle.y - rectangle.h;
		points[3].z = 0;

		
		
		projection->ProjectDepthToCamera(4, points, points3D);
		projection->ProjectCameraToColor(4, points, pointsDepth);
	}


	std::vector<PXCPoint3DF32> vertices(SIZE);
	CloudType::Ptr xyz_cloud(new CloudType(w, h));


	PXCImage::ImageData data;
	PXCCapture::Sample *sam = pp->QuerySample();
	sam->depth->AcquireAccess(PXCImage::ACCESS_READ, &data);
	projection->QueryVertices(sam->depth, vertices.data());
	sam->depth->ReleaseAccess(&data);
	projection->Release();

	pp->ReleaseFrame();

	int validP = 0;
	for (int i = 0; i < SIZE; i++) {
		if (convertPoint(vertices[i], xyz_cloud->points[i], points3D, _use_face)) {
			++validP;
		}
	}
	renderer->setCurrentFramePCL(actual_cloud);
	if (_use_face) {
		renderer->setCurrentFrameRectangle(points3D);
		renderer->setCurrentFrameRectangleRGB(pointsDepth);
	}
	//for (int i = 0; i < _cloud1->size(); i++) {
	//	assert(!std::isnan(_cloud1->points[i].x));
	//}
	if (_tracker_set == false) { //Use built-in algorithm
		QTime algorithmTime;
		algorithmTime.start();
		orientationIsValid = track();
		int elapsed = algorithmTime.elapsed();
		++_actual_frame;
		app->addIncrementalTimeMean(elapsed);
	}

	else { //Use algorithm in tracker and filters

		double radius_search = tracker->getParameter("radius_search").toDouble();
		bool subsampling_none = tracker->getParameter("subsample") == "none";
		bool subsampling_uniform = tracker->getParameter("subsample") == "uniform";
		bool subsampling_random = tracker->getParameter("subsample") == "random";

		QTime algorithmTime;
		algorithmTime.start();
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
			//Pass cloud to renderer (Or pass the filtered one to see differences)
			
			if (subsampling_uniform) {
				
				pcl::PointCloud<int> indicesIN;
				uniform_sampling.setInputCloud(cloud_filtered);
				uniform_sampling.setRadiusSearch(radius_search);
				uniform_sampling.compute(indicesIN);
				//pcl::copyPointCloud(*cloud_filtered, indicesIN.points, *first_cloud);
				pcl::copyPointCloud(*cloud_filtered, indicesIN.points, *first_cloud);
				pcl::copyPointCloud(*cloud_filtered, indicesIN.points, *last_cloud);
				
			}
			else if (subsampling_none) {
				pcl::copyPointCloud(*cloud_filtered, *first_cloud);
				pcl::copyPointCloud(*cloud_filtered, *last_cloud);
			}
			else if (subsampling_random) {
				pcl::copyPointCloud(*cloud_filtered, *first_cloud);
				pcl::copyPointCloud(*cloud_filtered, *last_cloud);
			}
			//Save in first_cloud the first frame, filtered and prepared :D
			//pcl::io::savePCDFileASCII("../clouds/rs/init_cloud.pcd", *cloud_filtered);
		}
		//Filter NANS for ICP
		xyz_cloud->is_dense = false;

		std::vector<int> indices;
		CloudType::Ptr _cloud1(new CloudType);
		pcl::removeNaNFromPointCloud(*xyz_cloud, *_cloud1, indices);

		//Filter points using filter object
		if (subsampling_uniform) {
			QTime filterTime;
			filterTime.start();
			pcl::PointCloud<int> indicesOUT;
			uniform_sampling.setInputCloud(_cloud1);
			uniform_sampling.setRadiusSearch(radius_search);
			uniform_sampling.compute(indicesOUT);
			pcl::copyPointCloud(*_cloud1.get(), indicesOUT.points, *actual_cloud);
			++_actual_frame;
			//pcl::io::savePCDFileASCII("../clouds/rs/actual_cloud_" + QString::number(_actual_frame).toStdString() + ".pcd", *actual_cloud);
			//pcl::copyPointCloud(*_cloud1.get(), *actual_cloud);
			int elapsedF = filterTime.elapsed();
			//Save frame info to file
			QFile file("../"+QString::number(radius_search)+"_uniform_info.txt");
			if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
				QTextStream stream(&file);
				stream << radius_search << " " << _cloud1->size() - actual_cloud->size() << " " << elapsedF << endl;
			}
		}
		else if (subsampling_none) {
			pcl::copyPointCloud(*_cloud1.get(), *actual_cloud);
		}
		else if (subsampling_random) {
			pcl::copyPointCloud(*_cloud1.get(), *actual_cloud);
		}
		


		//Pass cloud to algorithm
		if (_last_init) { //Compute if we have 2 frames (last_cloud is filled with the first frame)
			app->setPointsAnalyzed(actual_cloud->size(), first_cloud->size());
			if (actual_cloud->size() > 0) {
				
			}
			double fitness;
			orientationIsValid = tracker->compute(*last_cloud.get(), *actual_cloud.get(), _roll, _pitch, _yaw, fitness);
			app->setICPConverged(orientationIsValid, fitness);
			if (valid) {
				orientationIsValid = true;
			}
		}
		int elapsed = algorithmTime.elapsed();
		app->addIncrementalTimeMean(elapsed);
	}

	

	renderer->setFaceTracked(orientationIsValid);
	app->setFaceTracked(orientationIsValid);
	if (orientationIsValid) {
		app->setFaceAngles(_yaw, _pitch, _roll);
	}
}

void RealSense::stop() {
	pp->Close();
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
	//sample->depth->AcquireAccess(PXCImage::ACCESS_READ, PXCImage::PIXEL_FORMAT_RGB32, &depthData);
	int height = sample->color->QueryInfo().height;
	int width = sample->color->QueryInfo().width;

	//int height2 = sample->depth->QueryInfo().height;
	//int width2 = sample->depth->QueryInfo().width;
	// image planes are in data.planes[0-3] with pitch data.pitches[0-3]
	QImage colorImage(colorData.planes[0], width, height, QImage::Format_RGB32);
	//QImage depthImage(depthData.planes[0], width2, height2, QImage::Format_RGB32);
	image = colorImage;

	QDir dir("../tests/rs/last_test");
	if (!dir.exists()) {
		dir.mkpath(".");
		colorImage.save("../tests/rs/last_test/image_" + QString::number(_actual_frame) + ".png");
	}
	else {
		colorImage.save("../tests/rs/last_test/image_" + QString::number(_actual_frame)+".png");
	}
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
			//renderer->setCurrentFrameRectangleRGB(rectangle);
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
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
	focalLength = _device->QueryDepthFocalLengthMM();
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
convertPoint(const PXCPoint3DF32& src, T& tgt, bool useFace)
{
	static const float nan = std::numeric_limits<float>::quiet_NaN();
	if (src.z == 0 || src.z > 500)
		//Point is invalid
	{
		tgt.x = tgt.y = tgt.z = nan;
		return false;
		//tgt.x = tgt.y = tgt.z = 0;

	}
	else 
	{
		tgt.x = -src.x / 1000.0;
		tgt.y = src.y / 1000.0;
		tgt.z = src.z / 1000.0;
		return true;
	}
}



void RealSense::update() {

	float time_update = 0;
	float time_filter = 0;
	float time_icp = 0;
	float points_used = 0;
	float icp_fitness = 0;
	bool has_converged = 0;

	QTime updateTime;
	updateTime.start();
	//if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
	//	return;
	if (pp->AcquireFrame(true)<PXC_STATUS_NO_ERROR)
		return;

	outputData->Update();
	bool valid = false;
	/* Detection Structs */
	PXCFaceData::DetectionData *detectionData;

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
		}
	}
	pp->ReleaseFrame();
	QImage actualFrame;
	getFrameImage(actualFrame);
	renderer->initTexture(actualFrame);
	//renderer->setCurrentFrameToShow(actualFrame);
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

	pp->ReleaseFrame();
	renderer->setCurrentFramePCL(actual_cloud);
	int validP = 0;
	for (int i = 0; i < SIZE; i++) {
		if (convertPoint(vertices[i], xyz_cloud->points[i], _use_face)) {
			++validP;
		}
	}
		
	if (validP != 0) {//Then DO SOMETHING
		

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
				time_filter = elapsedF;
				//Save frame info to file for DOCUMENTATION
				//QFile file("../"+QString::number(radius_search)+"_uniform_info.txt");
				//if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
				//	QTextStream stream(&file);
				//	stream << radius_search << " " << _cloud1->size() - actual_cloud->size() << " " << elapsedF << endl;
				//}
			}
			else if (subsampling_none) {
				pcl::copyPointCloud(*_cloud1.get(), *actual_cloud);
			}
			else if (subsampling_random) {
				pcl::copyPointCloud(*_cloud1.get(), *actual_cloud);
			}



			//Pass cloud to algorithm
			if (_last_init) { //Compute if we have 2 frames (last_cloud is filled with the first frame)
				points_used = actual_cloud->size();
				app->setPointsAnalyzed(actual_cloud->size(), first_cloud->size());
				if (actual_cloud->size() > 0) {

				}
				double fitness;
				orientationIsValid = tracker->compute(*last_cloud.get(), *actual_cloud.get(), _roll, _pitch, _yaw, fitness);
				has_converged = orientationIsValid;
				icp_fitness = fitness;
				app->setICPConverged(orientationIsValid, fitness);
				if (valid) {
					orientationIsValid = true;
				}
			}
			int elapsed = algorithmTime.elapsed();
			time_icp = elapsed;
			app->addIncrementalTimeMean(elapsed);
		}



		renderer->setFaceTracked(orientationIsValid);
		app->setFaceTracked(orientationIsValid);
		if (orientationIsValid) {
			app->setFaceAngles(_yaw, _pitch, _roll);
		}
	}
	int elapsedUpdate = updateTime.elapsed();
	time_update = elapsedUpdate;
	app->setFilterTime(elapsedUpdate);

	if (_tracker_set) {
		QFile file("../info_gather_params.csv");
		if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
			QTextStream stream(&file);
			//Write data to file for DOCU 
			stream << tracker->getParameter("radius_search").toDouble() << ";" <<
				tracker->getParameter("epsilon").toDouble() << ";" <<
				tracker->getParameter("epsilon_transform").toDouble() << ";" <<
				tracker->getParameter("max_iter").toDouble() << ";" <<
				tracker->getParameter("max_corresp_dist").toDouble() << ";" <<
				QString((has_converged ? "TRUE" : "FALSE")) << ";" << icp_fitness << ";" << time_icp << ";" << time_filter << ";" << time_update << ";" << points_used << "\n";
	}
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

	//QDir dir("../tests/rs/last_test");
	//if (!dir.exists()) {
	//	dir.mkpath(".");
	//	colorImage.save("../tests/rs/last_test/image_" + QString::number(_actual_frame) + ".png");
	//}
	//else {
	//	colorImage.save("../tests/rs/last_test/image_" + QString::number(_actual_frame)+".png");
	//}
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
#include "MKinect.h"
#include "facetrackingapp.h"

MKinect::MKinect()
{
	
}


MKinect::~MKinect()
{
	stop();
}

void MKinect::init(FaceTrackingApp *app){
	this->app = app;
	_actual_frame = 0;
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return;
	}
	if (sensor) {
		HRESULT hResult = S_OK;
		sensor->get_CoordinateMapper(&mapper);

		sensor->Open();
		sensor->OpenMultiSourceFrameReader(
			FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
			&reader);
		
		hResult = sensor->get_BodyFrameSource(&pBodySource);
		if (FAILED(hResult)){
			std::cerr << "Error : IKinectSensor::get_BodyFrameSource()" << std::endl;
			return;
		}

		
		hResult = pBodySource->OpenReader(&pBodyReader);
		if (FAILED(hResult)){
			std::cerr << "Error : IBodyFrameSource::OpenReader()" << std::endl;
			return;
		}

		DWORD features = FaceFrameFeatures::FaceFrameFeatures_BoundingBoxInColorSpace
			| FaceFrameFeatures::FaceFrameFeatures_PointsInColorSpace
			| FaceFrameFeatures::FaceFrameFeatures_RotationOrientation
			| FaceFrameFeatures::FaceFrameFeatures_Happy
			| FaceFrameFeatures::FaceFrameFeatures_RightEyeClosed
			| FaceFrameFeatures::FaceFrameFeatures_LeftEyeClosed
			| FaceFrameFeatures::FaceFrameFeatures_MouthOpen
			| FaceFrameFeatures::FaceFrameFeatures_MouthMoved
			| FaceFrameFeatures::FaceFrameFeatures_LookingAway
			| FaceFrameFeatures::FaceFrameFeatures_Glasses
			| FaceFrameFeatures::FaceFrameFeatures_FaceEngagement;
		
		for (int count = 0; count < BODY_COUNT; count++){
			// Source
			hResult = CreateFaceFrameSource(sensor, 0, features, &pFaceSource[count]);
			if (FAILED(hResult)){
				std::cerr << "Error : CreateFaceFrameSource" << std::endl;
				return;
			}

			// Reader
			hResult = pFaceSource[count]->OpenReader(&pFaceReader[count]);
			if (FAILED(hResult)){
				std::cerr << "Error : IFaceFrameSource::OpenReader()" << std::endl;
				return;
			}
		}
		return;
	}
	else {
		return;
	}
	
}
void MKinect::stop(){
	if (FAILED(GetDefaultKinectSensor(&sensor))) {
		return;
	}
	if (sensor) {
		sensor->Close();
		reader->Release();
	}
	else {
		return;
	}
}
void MKinect::getFrameImage(QImage &image){
	

}
void MKinect::getDepthPoints(){

}

void MKinect::getKinectData(QImage &image) {
	IMultiSourceFrame* frame = NULL;
	while (!SUCCEEDED(reader->AcquireLatestFrame(&frame))) {

	}
	//getDepthData(frame, dest);
	getColorData(frame, image);
}

void MKinect::getDepthData(IMultiSourceFrame* frame) {
	IDepthFrame* depthframe;
	IDepthFrameReference* frameref = NULL;
	frame->get_DepthFrameReference(&frameref);
	frameref->AcquireFrame(&depthframe);
	if (frameref) frameref->Release();
	if (!depthframe) return;
	// Get data from frame
	unsigned int sz;
	unsigned short* buf;
	depthframe->AccessUnderlyingBuffer(&sz, &buf);

	mapper->MapDepthFrameToCameraSpace(
		KinectColorWidth*KinectColorHeight, buf,        // Depth frame data and size of depth frame
		KinectColorWidth*KinectColorHeight, depth2xyz); // Output CameraSpacePoint array and size
	// Process depth frame data...

	if (depthframe) depthframe->Release();
}

void MKinect::getColorData(IMultiSourceFrame* frame, QImage& dest) {
	IColorFrame* colorframe;
	IColorFrameReference* frameref = NULL;
	frame->get_ColorFrameReference(&frameref);
	frameref->AcquireFrame(&colorframe);
	if (frameref) frameref->Release();
	if (!colorframe) return;

	// Process color frame data...
	colorframe->CopyConvertedFrameDataToArray(KinectColorWidth*KinectColorHeight * 4, data, ColorImageFormat_Bgra);
	QImage colorImage(data, KinectColorWidth, KinectColorHeight, QImage::Format_RGB32);
	//QImage depthImage(depthData.planes[0], width2, height2, QImage::Format_RGB32);
	dest = colorImage;
	QDir dir("../tests/k2/last_test");
	if (!dir.exists()) {
		dir.mkpath(".");
		colorImage.save("../tests/k2/last_test/image_" + QString::number(_actual_frame) + ".png", 0);
	}
	else {
		colorImage.save("../tests/k2/last_test/image_" + QString::number(_actual_frame) + ".png", 0);
	}
	if (colorframe) colorframe->Release();
}

// Quote from Kinect for Windows SDK v2.0 Developer Preview - Samples/Native/FaceBasics-D2D, and Partial Modification
// ExtractFaceRotationInDegrees is: Copyright (c) Microsoft Corporation. All rights reserved.
inline void ExtractFaceRotationInDegrees(const Vector4* pQuaternion, int* pPitch, int* pYaw, int* pRoll)
{
	double x = pQuaternion->x;
	double y = pQuaternion->y;
	double z = pQuaternion->z;
	double w = pQuaternion->w;

	// convert face rotation quaternion to Euler angles in degrees
	*pPitch = static_cast<int>(std::atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / M_PI * 180.0f);
	*pYaw = static_cast<int>(std::asin(2 * (w * y - x * z)) / M_PI * 180.0f);
	*pRoll = static_cast<int>(std::atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / M_PI * 180.0f);
}

void MKinect::update(){
	QImage actualFrame;
	getKinectData(actualFrame);
	++_actual_frame;
	renderer->initTexture(actualFrame);
	if (_tracker_set) {

	}
	else {
		QTime algorithmTime;
		algorithmTime.start();
		track();
		int elapsed = algorithmTime.elapsed();
		app->addIncrementalTimeMean(elapsed);
	}
	app->setFaceAngles(_yaw, _pitch, _roll);
}
bool MKinect::track(){
	HRESULT hResult = S_OK;
	// Body Frame
	IBodyFrame* pBodyFrame = nullptr;
	hResult = pBodyReader->AcquireLatestFrame(&pBodyFrame);
	if (SUCCEEDED(hResult)){
		IBody* pBody[BODY_COUNT] = { 0 };
		hResult = pBodyFrame->GetAndRefreshBodyData(BODY_COUNT, pBody);
		if (SUCCEEDED(hResult)){
			for (int count = 0; count < BODY_COUNT; count++){
				BOOLEAN bTracked = false;
				hResult = pBody[count]->get_IsTracked(&bTracked);
				if (SUCCEEDED(hResult) && bTracked){
					/*// Joint
					Joint joint[JointType::JointType_Count];
					hResult = pBody[count]->GetJoints( JointType::JointType_Count, joint );
					if( SUCCEEDED( hResult ) ){
					for( int type = 0; type < JointType::JointType_Count; type++ ){
					ColorSpacePoint colorSpacePoint = { 0 };
					pCoordinateMapper->MapCameraPointToColorSpace( joint[type].Position, &colorSpacePoint );
					int x = static_cast<int>( colorSpacePoint.X );
					int y = static_cast<int>( colorSpacePoint.Y );
					if( ( x >= 0 ) && ( x < width ) && ( y >= 0 ) && ( y < height ) ){
					cv::circle( bufferMat, cv::Point( x, y ), 5, static_cast<cv::Scalar>( color[count] ), -1, CV_AA );
					}
					}
					}*/

					// Set TrackingID to Detect Face
					UINT64 trackingId = _UI64_MAX;
					hResult = pBody[count]->get_TrackingId(&trackingId);
					if (SUCCEEDED(hResult)){
						pFaceSource[count]->put_TrackingId(trackingId);
					}
				}
			}
		}
		for (int count = 0; count < BODY_COUNT; count++){
			if (pBody[count]) pBody[count]->Release();
		}
	}
	if (pBodyFrame) pBodyFrame->Release();
	// Face Frame
	for (int count = 0; count < BODY_COUNT; count++){
		IFaceFrame* pFaceFrame = nullptr;
		hResult = pFaceReader[count]->AcquireLatestFrame(&pFaceFrame);
		if (SUCCEEDED(hResult) && pFaceFrame != nullptr){
			BOOLEAN bFaceTracked = false;
			hResult = pFaceFrame->get_IsTrackingIdValid(&bFaceTracked);
			if (SUCCEEDED(hResult) && bFaceTracked){
				IFaceFrameResult* pFaceResult = nullptr;
				hResult = pFaceFrame->get_FaceFrameResult(&pFaceResult);
				if (SUCCEEDED(hResult) && pFaceResult != nullptr){
					std::vector<std::string> result;
					// Face Rotation
					Vector4 faceRotation;
					hResult = pFaceResult->get_FaceRotationQuaternion(&faceRotation);
					if (SUCCEEDED(hResult)){
						int pitch, yaw, roll;
						ExtractFaceRotationInDegrees(&faceRotation, &pitch, &yaw, &roll);
						_yaw = yaw;
						_pitch = pitch;
						_roll = roll;
					}
				}
				if (pFaceResult) pFaceResult->Release();
			}
		}
		if (pFaceFrame) pFaceFrame->Release();
	}

	return true;
}

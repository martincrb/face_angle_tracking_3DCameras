#include "facetrackingapp.h"

FaceTrackingApp::FaceTrackingApp(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
	_cam_init = false;
	_selected_alg = "Camera Algorithm";
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(onNewFrame()));

	//Detect connected cameras
	QStringList cameras = getConnectedCameras();
	QStringList camerasSimplified;
	bool sr300 = false;
	for (QString cameraStr : cameras) {
		if (cameraStr.contains("SR300") && !sr300) {
			camerasSimplified.append("Intel(R) RealSense(TM) Camera SR300");
			sr300 = true;
		}
	}
	camerasModel = new QStringListModel(camerasSimplified);

	QStringList algorithms;
	algorithms.append("Camera Algorithm");
	algorithms.append("ICP Algorithm");
	algModel = new QStringListModel(algorithms);
	ui.alg_list->setModel(algModel);
	
	ui.alg_list->setCurrentIndex(algModel->index(0));
	ui.cam_list->setModel(camerasModel);
	ui.cam_list->show();

	ui.pushButton->setEnabled(false);
	ui.pushButton_2->setEnabled(false);
	ui.pushButton_3->setEnabled(false);

}

void FaceTrackingApp::setSelectedCamera(QModelIndex idx) {
	_selected_camera = camerasModel->data(idx, 0).toString();
}

void FaceTrackingApp::setSelectedAlgorithm(QModelIndex idx) {
	_selected_alg = algModel->data(idx, 0).toString();
}

void FaceTrackingApp::permitButtons() {
	ui.pushButton->setEnabled(true);
	ui.pushButton_2->setEnabled(true);
	ui.pushButton_3->setEnabled(true);
}
void FaceTrackingApp::setFaceTracked(bool tracked) {
	if (tracked) {
		ui.face_text->setText("Face detected: TRUE");
	}
	else {
		ui.face_text->setText("Face detected: FALSE");
	}
}
void FaceTrackingApp::setFaceAngles(float yaw, float pitch, float roll) {
	ui.label_6->setNum(yaw);
	ui.label_7->setNum(pitch);
	ui.label_8->setNum(roll);
}

void FaceTrackingApp::changeMainRenderModeToRGB() {
	ui.camviewer->changeMode("RGB");
	if (ui.radioButton_2->isChecked()) {
		ui.radioButton_2->setChecked(false);
	}
}

void FaceTrackingApp::changeMainRenderModeToPCL() {
	ui.camviewer->changeMode("PCL");
	if (ui.radioButton->isChecked()) {
		ui.radioButton->setChecked(false);
	}
}

FaceTrackingApp::~FaceTrackingApp()
{

}

void FaceTrackingApp::onStart()
{

}
void FaceTrackingApp::onStop()
{

}
void FaceTrackingApp::onNewFrame()
{
	if (_cam_init) {
		camera->update();
	}
}

void FaceTrackingApp::initSelected()
{
	
	ui.statusBar->showMessage("Initializing 3D Camera: " + _selected_camera);
	//TODO: Select camera based on selection
	if (_selected_camera == "Intel(R) RealSense(TM) Camera SR300") {
		camera = new RealSense();
	}
	ui.cam_list->setEnabled(false);
	ui.alg_list->setEnabled(false);
	camera->setRenderer(ui.camviewer);
	if (_selected_alg == "Camera Algorithm") {
		camera->setTracker(nullptr); //Use built-in algorithm
	}
	else if (_selected_alg == "ICP Algorithm") {
		algorithm = new IterativeClosestPoint();
		camera->setTracker(algorithm);
	}

	initAlgorithmParameters();
	camera->init(this);
	timer->start(1000 / 24);
	_cam_init = true;
	ui.statusBar->showMessage("Intel(R) RealSense(TM) Camera SR300 Running with " + _selected_alg);
}

void  FaceTrackingApp::initAlgorithmParameters() {
	//Read parameters from GUI
	QMap<QString, QString> parameters;

	//Set params on algorithm 
	if (camera->trackerIsSet()) {
		algorithm->setParameters(parameters);
		algorithm->initializeParameters();
	}
}

void FaceTrackingApp::stopCamera() {
	timer->stop();
	camera->stop();
	_cam_init = false;
	ui.cam_list->setEnabled(true);
	ui.alg_list->setEnabled(true);
	ui.label_6->setText("-");
	ui.label_7->setText("-");
	ui.label_8->setText("-");
}

QStringList FaceTrackingApp::getConnectedCameras()
{
	QStringList cameras;
	MSUtils::getInputDevices(cameras);
	return cameras;
}

#include "facetrackingapp.h"

FaceTrackingApp::FaceTrackingApp(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	
	_cam_init = false;
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(onNewFrame()));

	//Detect connected cameras
	QStringList cameras = getConnectedCameras();
	QStringListModel *camerasModel = new QStringListModel(cameras);

	ui.cam_list->setModel(camerasModel);
	ui.cam_list->show();
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
	
	ui.statusBar->showMessage("Initializing 3D Camera");
	//TODO: Select camera based on selection
	camera = new RealSense();
	camera->setRenderer(ui.camviewer);
	camera->init();
	timer->start(1000 / 24);
	_cam_init = true;
	ui.statusBar->showMessage("Intel Real Sense Running");
}

void FaceTrackingApp::stopCamera() {
	timer->stop();

}

QStringList FaceTrackingApp::getConnectedCameras()
{
	QStringList cameras;
	MSUtils::getInputDevices(cameras);
	return cameras;
}

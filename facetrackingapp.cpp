#include "facetrackingapp.h"

FaceTrackingApp::FaceTrackingApp(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	_elapsedFrames = 0;
	_last_mean_frametime = 0;
	ui.lineEdit->setText("0.005");
	ui.lineEdit_2->setText("0.005");
	_actualAngle = 0;
	_actualFrame = 0;
	_cam_init = false;
	_test_started = false;
	_selected_alg = "Camera Algorithm";
	timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(onNewFrame()));

	ui.widget->addGraph();
	ui.widget->graph(0)->setPen(QPen(Qt::blue));

	ui.widget->addGraph();
	ui.widget->graph(1)->setPen(QPen(Qt::red));
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

	
	//Generate test 1
	//Save frame info to file
	/*
	QFile file("../test_01.txt");
	if (file.open(QIODevice::WriteOnly | QIODevice::Append)) {
		QTextStream stream(&file);
		for (int j = 0; j < 10; ++j) {
			for (int i = 0; i < 90; i += 4) {
				stream << i << " ";
			}
			for (int i = 90; i > 0; i -= 4) {
				stream << i << " ";
			}
		}
	}
	*/
}

#include <QFileDialog>
void FaceTrackingApp::startTest() {
	auto fileName = QFileDialog::getOpenFileName(this,
		tr("Open Test File"), "/tests", tr("Test Files (*.test)"));
	if (fileName != NULL) {
		QFile file(fileName);
		if (file.open(QIODevice::ReadOnly)) {
			QString angles = file.readLine();
			_actual_test = angles.split(" ");
			_actual_idx = 0;
		}
		testPlatform.init(L"COM7");
		//testPlatform.sendData("start_test");
		initSelected();
		_test_started = true;
	}
}

void FaceTrackingApp::addIncrementalTimeMean(double time) {
	++_elapsedFrames;
	//http://math.stackexchange.com/questions/106700/incremental-averageing
	double new_mean = ((_elapsedFrames - 1)*_last_mean_frametime + time) / _elapsedFrames;
	ui.label_12->setText("Mean time per frame: " + QString::number(new_mean));
}
void FaceTrackingApp::setPointsAnalyzed(int points) {
	ui.label_13->setText("Points Analyzed: " + QString::number(points));
}

void FaceTrackingApp::setSelectedCamera(QModelIndex idx) {
	_selected_camera = camerasModel->data(idx, 0).toString();
}

void FaceTrackingApp::setSelectedAlgorithm(QModelIndex idx) {
	_selected_alg = algModel->data(idx, 0).toString();
	if (_selected_alg == "Camera Algorithm") {
		ui.groupBox_2->setEnabled(false);
	}
	else {
		ui.groupBox_2->setEnabled(true);
	}
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
	
	//Update plots
	_plot_detected.append(yaw);
	_plot_frames.append(_actualFrame++);

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

void FaceTrackingApp::updatePlot(bool test) {
	//Prepare plot
	
	ui.widget->yAxis->setLabel("Angles");
	ui.widget->xAxis->setLabel("Frame");
	ui.widget->yAxis->setRange(-90, 90);
	ui.widget->xAxis->setRange(0, _plot_frames.size() - 1);
	ui.widget->graph(0)->setData(_plot_frames, _plot_detected);
	ui.widget->graph(0)->setName("Detected angle");
	if (test) {
		ui.widget->graph(1)->setVisible(true);
		ui.widget->graph(1)->setData(_plot_frames, _plot_shouldbe);
		ui.widget->graph(1)->setName("Motor angle");
	}
	else {
		ui.widget->graph(1)->setVisible(false);
	}
	ui.widget->replot();
}
void FaceTrackingApp::onNewFrame()
{
	if (_cam_init) {
		bool update = true;
		if (_test_started) {
			QString angle = _actual_test.at(_actual_idx);
			testPlatform.sendData(angle);
			_plot_shouldbe.append(angle.toDouble());
			++_actual_idx;
			if (_actual_idx >= _actual_test.size()) {
				//testPlatform.sendData("stop_test");
				_test_started = false;
				_actual_idx = 0;
				stopCamera();
				update = false;
			}
		}
		if (update) {
			camera->update();
			updatePlot(_test_started);
		}
		//if (_actualAngle >= 180) {
		//	_actualAngle = 0;
		//}
		//testPlatform.sendData(QString::number((int)_actualAngle));
		//++_actualAngle;
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
	camera->useFace(ui.use_face_check->isChecked());
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
	_plot_shouldbe.clear();
	_plot_detected.clear();
	_plot_frames.clear();
	_actualFrame = 0;
}

void  FaceTrackingApp::initAlgorithmParameters() {
	//Read parameters from GUI
	QMap<QString, QString> parameters;
	parameters.insert("epsilon", ui.lineEdit->text()); //Read epsilon parameter

	bool no_subsample = ui.radioButton_3->isChecked(); //Read subsample method (only one will be true)
	bool uniform_subsample = ui.radioButton_4->isChecked();
	if (no_subsample) {
		parameters.insert("subsample", "none");
	}
	else if (uniform_subsample) {
		parameters.insert("subsample", "uniform");
		parameters.insert("radius_search", ui.lineEdit_2->text());
	}
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
	ui.label_13->setText("Points Analyzed: -");
	_elapsedFrames = 0;
	_last_mean_frametime = 0;
	if (_test_started) {
		testPlatform.close();
	}
}

QStringList FaceTrackingApp::getConnectedCameras()
{
	QStringList cameras;
	MSUtils::getInputDevices(cameras);
	return cameras;
}

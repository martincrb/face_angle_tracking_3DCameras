#include "facetrackingapp.h"

FaceTrackingApp::FaceTrackingApp(QWidget *parent)
	: QMainWindow(parent)
{
	ui.setupUi(this);
	_elapsedFrames = 0;
	_last_mean_frametime = 0;
	ui.lineEdit->setText("0.1"); //Euclidean epsilon
	ui.lineEdit_2->setText("0.008"); //radius subsampling search
	ui.lineEdit_3->setText("0.01"); //transformation epsilon
	ui.lineEdit_4->setText("50"); //max iter
	ui.lineEdit_5->setText("4"); //max correspondence distance
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

	ui.widget_2->addGraph();
	ui.widget_2->graph(0)->setPen(QPen(Qt::blue));

	ui.widget_3->addGraph();
	ui.widget_3->graph(0)->setPen(QPen(Qt::blue));


	//Detect connected cameras
	QStringList cameras = getConnectedCameras();
	QStringList camerasSimplified;
	bool sr300 = false;
	bool kinect2 = false;
	for (QString cameraStr : cameras) {
		if (cameraStr.contains("SR300") && !sr300) {
			camerasSimplified.append("Intel(R) RealSense(TM) Camera SR300");
			sr300 = true;
		}
		
	}
	camerasSimplified.append("Microsoft Kinect 2");
	kinect2 = true;
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
	ui.label_12->setText("ICP Time (mean): " + QString::number(new_mean));
}

void FaceTrackingApp::setFilterTime(double time) {
	ui.label_22->setText("Update time: " + QString::number(time));
}


void FaceTrackingApp::setPointsAnalyzed(int points, int points2) {
	ui.label_13->setText("Points Analyzed: " + QString::number(points) + " vs " + QString::number(points2));
}

void FaceTrackingApp::setICPConverged(bool conv, double fitness){
	if (conv) {
		ui.label_17->setText("ICP converged: TRUE");
		ui.label_18->setText("ICP Fitness: " + QString::number(fitness));
	}
	else {
		ui.label_17->setText("ICP converged: FALSE");
		ui.label_18->setText("ICP Fitness: -");
	}
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
	_plot_detected_r.append(roll);
	_plot_detected_p.append(pitch);
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
	//Yaw Plot
	ui.widget->yAxis->setLabel("Yaw");
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

	//Pitch plot
	ui.widget_2->yAxis->setLabel("Pitch");
	ui.widget_2->xAxis->setLabel("Frame");
	ui.widget_2->yAxis->setRange(-90, 90);
	ui.widget_2->xAxis->setRange(0, _plot_frames.size() - 1);
	ui.widget_2->graph(0)->setData(_plot_frames, _plot_detected_p);
	ui.widget_2->graph(0)->setName("Detected angle");
	
	ui.widget_2->replot();

	//Roll plot
	ui.widget_3->yAxis->setLabel("Roll");
	ui.widget_3->xAxis->setLabel("Frame");
	ui.widget_3->yAxis->setRange(-90, 90);
	ui.widget_3->xAxis->setRange(0, _plot_frames.size() - 1);
	ui.widget_3->graph(0)->setData(_plot_frames, _plot_detected_r);
	ui.widget_3->graph(0)->setName("Detected angle");
	
	ui.widget_3->replot();
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
	else if (_selected_camera == "Microsoft Kinect 2") {
		camera = new MKinect();
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
	_plot_shouldbe.clear();
	_plot_detected.clear();
	_plot_detected_p.clear();
	_plot_detected_r.clear();
	_plot_frames.clear();
	_actualFrame = 0;
}

void  FaceTrackingApp::initAlgorithmParameters() {
	//Read parameters from GUI
	QMap<QString, QString> parameters;
	parameters.insert("epsilon", ui.lineEdit->text()); //Read epsilon parameter
	parameters.insert("epsilon_transform", ui.lineEdit_3->text()); //Read epsilon transformation parameter
	parameters.insert("max_iter", ui.lineEdit_4->text()); //Read epsilon transformation parameter
	parameters.insert("max_corresp_dist", ui.lineEdit_5->text()); //Read epsilon transformation parameter

	parameters.insert("focal_length", QString::number(camera->getFocalLength()));
	bool no_subsample = ui.radioButton_3->isChecked(); //Read subsample method (only one will be true)
	bool uniform_subsample = ui.radioButton_4->isChecked();
	if (no_subsample) {
		parameters.insert("subsample", "none");
	}
	else if (uniform_subsample) {
		parameters.insert("subsample", "uniform");
		parameters.insert("radius_search", ui.lineEdit_2->text());
	}
	if (ui.checkBox_2->isChecked()) {
		parameters.insert("normals", "true");
	}
	else {
		parameters.insert("normals", "false");
	}
	if (ui.checkBox->isChecked()) {
		parameters.insert("onetoone", "true");
	}
	else {
		parameters.insert("onetoone", "false");
	}
	if (ui.checkBox_3->isChecked()) {
		parameters.insert("boundary", "true");
	}
	else {
		parameters.insert("boundary", "false");
	}
	if (ui.checkBox_5->isChecked()) {
		parameters.insert("rej_dist", "true");
		parameters.insert("rej_dist_threshold", ui.lineEdit_6->text());
	}
	else {
		parameters.insert("rej_dist", "false");
	}
	if (ui.checkBox_4->isChecked()) {
		parameters.insert("rej_median", "true");
		parameters.insert("rej_median_threshold", ui.lineEdit_7->text());
	}
	else {
		parameters.insert("rej_median", "false");
	}
	if (ui.checkBox_6->isChecked()) {
		parameters.insert("rej_normals", "true");
		parameters.insert("rej_normals_threshold", ui.lineEdit_8->text());
	}
	else {
		parameters.insert("rej_normals", "false");
	}
	if (ui.use_projection->isChecked()) {
		parameters.insert("correspondence", "projection");
	}
	if (ui.radioButton_5->isChecked()) {
		parameters.insert("correspondence", "nearest");
	}
	//Set params on algorithm 
	if (camera->trackerIsSet()) {
		algorithm->setParameters(parameters);
		algorithm->initializeParameters();
	}
}

void FaceTrackingApp::setMessage(QString message) {
	ui.statusBar->showMessage(message);
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
	QString folder = "rs";
	if (_selected_camera == "Microsoft Kinect 2") folder = "k2";
	ui.widget->savePng("../tests/" + folder + "/last_test/" + _selected_alg + "yaw.png", 600, 400);
	ui.widget_2->savePng("../tests/" + folder + "/last_test/" + _selected_alg + "pitch.png", 600, 400);
	ui.widget_3->savePng("../tests/" + folder + "/last_test/" + _selected_alg + "roll.png", 600, 400);
}

void FaceTrackingApp::saveYawPlot() {
	//Yaw Plot
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Plot"),
		"",
		tr("Images (*.png)"));
	ui.widget->savePng(fileName, 600, 400);

}
void FaceTrackingApp::savePitchPlot() {

	//Pitch plot
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Plot"),
		"",
		tr("Images (*.png)"));
	ui.widget_2->savePng(fileName, 600, 400);

	
}
void FaceTrackingApp::saveRollPlot() {

	//Roll plot
	QString fileName = QFileDialog::getSaveFileName(this, tr("Save Plot"),
		"",
		tr("Images (*.png)"));
	ui.widget_3->savePng(fileName, 600, 400);
}

QStringList FaceTrackingApp::getConnectedCameras()
{
	QStringList cameras;
	MSUtils::getInputDevices(cameras);
	return cameras;
}

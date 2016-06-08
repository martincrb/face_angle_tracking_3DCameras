#pragma once
#include "MSUtils.h"
#include "qstringlistmodel.h"
#include <qtimer.h>
#include <QtWidgets/QMainWindow>
#include "Camera3D.h"
#include "RealSense.h"
#include "IterativeClosestPoint.h"
#include "ui_facetrackingapp.h"
#include "Arduino.h"
#include <iostream>
#include <qfile.h>
class FaceTrackingApp : public QMainWindow
{
	Q_OBJECT

public:
	FaceTrackingApp(QWidget *parent = 0);
	~FaceTrackingApp();
	
	void setFaceAngles(float yaw, float pitch, float roll);
	void setFaceTracked(bool tracked);


public slots:
	void onStart();
	void onStop();
	void onNewFrame();
	void permitButtons();
	void setSelectedCamera(QModelIndex idx);
	void setSelectedAlgorithm(QModelIndex idx);
	void initSelected();
	void stopCamera();
	void changeMainRenderModeToRGB();
	void changeMainRenderModeToPCL();
	void addIncrementalTimeMean(double time);
	void setPointsAnalyzed(int points);
	void startTest();
	void updatePlot(bool test);
private:
	Ui::FaceTrackingAppClass ui;
	void initAlgorithmParameters();
	QStringListModel *camerasModel;
	QStringListModel *algModel;
	QString _selected_camera;
	QString _selected_alg;
	QTimer *timer;
	Camera3D * camera;
	TrackingAlgorithm *algorithm;
	QStringList getConnectedCameras();
	QStringList _actual_test;
	int _actual_idx;
	Arduino testPlatform;

	QVector<double> _plot_detected;
	QVector<double> _plot_shouldbe;
	QVector<double> _plot_frames;
	int _actualFrame;
	int _actualAngle;
	int _elapsedFrames;
	double _last_mean_frametime;
	bool _cam_init;
	bool _test_started;
};

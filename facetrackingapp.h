#pragma once
#include "MSUtils.h"
#include "qstringlistmodel.h"
#include <qtimer.h>
#include <QtWidgets/QMainWindow>
#include "Camera3D.h"
#include "RealSense.h"
#include "IterativeClosestPoint.h"
#include "ui_facetrackingapp.h"

#include <iostream>


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

	bool _cam_init;
};

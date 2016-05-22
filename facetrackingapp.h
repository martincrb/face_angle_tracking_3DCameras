#pragma once
#include "MSUtils.h"
#include "qstringlistmodel.h"
#include <qtimer.h>
#include <QtWidgets/QMainWindow>
#include "Camera3D.h"
#include "RealSense.h"
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
	void initSelected();
	void stopCamera();
private:
	Ui::FaceTrackingAppClass ui;
	QStringListModel *camerasModel;
	QString _selected_camera;
	QTimer *timer;
	Camera3D * camera;
	QStringList getConnectedCameras();

	bool _cam_init;
};

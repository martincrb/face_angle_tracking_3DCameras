#ifndef FACETRACKINGAPP_H
#define FACETRACKINGAPP_H

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
	
public slots:
	void onStart();
	void onStop();
	void onNewFrame();
	void initSelected();
	void stopCamera();
private:
	Ui::FaceTrackingAppClass ui;
	
	QTimer *timer;
	Camera3D * camera;
	QStringList getConnectedCameras();

	bool _cam_init;
};

#endif // FACETRACKINGAPP_H

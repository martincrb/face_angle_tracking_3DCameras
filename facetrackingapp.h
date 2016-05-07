#ifndef FACETRACKINGAPP_H
#define FACETRACKINGAPP_H

#include <QtWidgets/QMainWindow>
#include "ui_facetrackingapp.h"

class FaceTrackingApp : public QMainWindow
{
	Q_OBJECT

public:
	FaceTrackingApp(QWidget *parent = 0);
	~FaceTrackingApp();

private:
	Ui::FaceTrackingAppClass ui;
};

#endif // FACETRACKINGAPP_H

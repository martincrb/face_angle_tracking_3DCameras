#include "facetrackingapp.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	FaceTrackingApp w;
	w.show();
	return a.exec();
}

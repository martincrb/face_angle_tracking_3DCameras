#pragma once
#include "TrackingAlgorithm.h"
#include "GLWidget.h"
#include <qimage.h>
#define MAX_FACES  1

class Camera3D
{
public:
	Camera3D();
	~Camera3D();
	virtual void init() = 0;
	virtual void stop() = 0;
	virtual void getFrameImage(QImage &image) = 0;
	virtual void getDepthPoints() = 0;
	virtual void getFaceOrientation() = 0;
	virtual void getFaceOrientation(TrackingAlgorithm *tA) = 0;
	virtual void update() = 0;
	void setRenderer(GLWidget *renderer);
protected:
	GLWidget *renderer;
};


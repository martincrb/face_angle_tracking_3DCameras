#include "Camera3D.h"


Camera3D::Camera3D()
{
}


Camera3D::~Camera3D()
{
}

void Camera3D::setRenderer(GLWidget *renderer) {
	this->renderer = renderer;
}
float Camera3D::getFocalLength() {
	return focalLength;
}
void Camera3D::useFace(bool face) {
	_use_face = face;
}
bool Camera3D::trackerIsSet() {
	return _tracker_set;
}
void Camera3D::setTracker(TrackingAlgorithm *tracker) {
	if (tracker != nullptr) {
		_tracker_set = true;
		this->tracker = tracker;
	}
	else {
		_tracker_set = false;
	}
}
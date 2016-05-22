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
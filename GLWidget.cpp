#include "GLWidget.h"
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <QGLWidget>

#define TIME 10
#define MARGIN 0.0

GLWidget::GLWidget(QWidget *parent)
	: QOpenGLWidget(parent)
{
	//Add things here
	QSurfaceFormat format;
	_mode = "RGB";
	format.setDepthBufferSize(24);
	format.setSamples(4);
	setFormat(format);
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	//timer->start(10);
	_zoom = 0.5;
	timer->start(TIME);
	_face_tracked = false;
	xyz_cloud = CloudType::Ptr(new CloudType(640, 480));
	//xyz_cloud->points.resize(640 * 480);
	
}


GLWidget::~GLWidget()
{
	clean();
}

void GLWidget::clean()
{
	//delete goes here
	glDeleteTextures(1, &textureID);
	//delete texture;
}

void GLWidget::initializeGL()
{
	initializeOpenGLFunctions();
	
	glClearColor(0.0f, 0.0f, 1.0f, 1.0f);

}

void GLWidget::changeMode(QString mode) {
	_mode = mode;
}

void GLWidget::initTexture(const QImage& texture) {
	glEnable(GL_TEXTURE_2D);
	glGenTextures(1, &textureID);
	glBindTexture(GL_TEXTURE_2D, textureID);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

	gldata = QGLWidget::convertToGLFormat(texture);

	glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, gldata.width(),
		gldata.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE,
		gldata.bits());
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

	glClearColor(0.0f, 0.0f, 1.0f, 1.0f);
	glDisable(GL_TEXTURE_2D);
}

void GLWidget::paintRGB() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	if (_face_tracked) {
		glColor3f(0.0, 1.0, 0.0);
	}
	else {
		glColor3f(1.0, 0.0, 0.0);
	}
	glBegin(GL_QUADS);
	glVertex3f(-1, -1, -1);
	glVertex3f(1, -1, -1);
	glVertex3f(1, 1, -1);
	glVertex3f(-1, 1, -1);
	glEnd();
	glColor3f(1.0, 1.0, 1.0);
	glEnable(GL_TEXTURE_2D);

	glBindTexture(GL_TEXTURE_2D, textureID);
	/*
	glBegin(GL_TRIANGLES);
	//glColor3f(1.0, 0.0, 0.0);
	glVertex3f(-0.5, -0.5, 0); glVertex2d(0, 0);
	//glColor3f(0.0, 1.0, 0.0);
	glVertex3f(0.5, -0.5, 0); glVertex2d(texture->width(), 0);
	//glColor3f(0.0, 0.0, 1.0);
	glVertex3f(0.0, 0.5, 0); glVertex2d(0, texture->height());
	glEnd();
	*/
	glBegin(GL_QUADS);
	glTexCoord2f(0, 0); glVertex3f(-1 + MARGIN, -1 + MARGIN, -1 + MARGIN);
	glTexCoord2f(1, 0); glVertex3f(1 - MARGIN, -1 + MARGIN, -1 + MARGIN);
	glTexCoord2f(1, 1); glVertex3f(1 - MARGIN, 1 - MARGIN, -1 + MARGIN);
	glTexCoord2f(0, 1); glVertex3f(-1 + MARGIN, 1 - MARGIN, -1 + MARGIN);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	PXCPoint3DF32 v1;
	PXCPoint3DF32 v2;
	PXCPoint3DF32 v3;
	PXCPoint3DF32 v4;
	v1.x = (2 * (float)_rectangleRGB[0].x / 640) - 1;
	v1.y = (2 * (float)_rectangleRGB[0].y / 480) - 1;

	v2.x = (2 * (float)_rectangleRGB[1].x / 640) - 1;
	v2.y = (2*(float)_rectangleRGB[1].y / 480) - 1;

	v3.x = (2*(float)_rectangleRGB[2].x / 640) - 1;
	v3.y = (2 * (float)_rectangleRGB[2].y / 480) - 1;

	v4.x = (2 * (float)_rectangleRGB[3].x / 640) - 1;
	v4.y = (2 * (float)_rectangleRGB[3].y / 480) - 1;

	glColor3f(1.0, 0.0, 1.0);
	glBegin(GL_LINES);
	glVertex3f(v1.x, v1.y, -1 + 2 * MARGIN);
	glVertex3f(v2.x, v2.y, -1 + 2 * MARGIN);

	glVertex3f(v2.x, v2.y, -1 + 2 * MARGIN);
	glVertex3f(v3.x, v3.y, -1 + 2 * MARGIN);

	glVertex3f(v3.x, v3.y, -1 + 2 * MARGIN);
	glVertex3f(v4.x, v4.y, -1 + 2 * MARGIN);

	glVertex3f(v4.x, v4.y, -1 + 2 * MARGIN);
	glVertex3f(v1.x, v1.y, -1 + 2 * MARGIN);

	glEnd();
	//glBegin(GL_LINE);
	//glVertex3f(-0.9, -0.9, -1 + 2 * MARGIN); glVertex3f(0.9, 0.9, -1 + 2 * MARGIN);
	//glVertex3f(v1.x, v1.y, -1 + 2 * MARGIN); glVertex3f(v2.x, v2.y, -1 + 2 * MARGIN);
	//glVertex3f(v2.x, v2.y, -1 + 2 * MARGIN); glVertex3f(v3.x, v3.y, -1 + 2 * MARGIN);

	//glEnd();



	/*
	glTexCoord2d(0, 0);// glVertex2d(0, 0);
	glTexCoord2d(0.6, 0);// glVertex2d(gldata.width(), 0);
	glTexCoord2d(0.6, 0.6);// glVertex2d(gldata.width(), gldata.height());
	glTexCoord2d(0, 0.6);// glVertex2d(0, gldata.height());
	glEnd();

	*/
}

void GLWidget::paintPCL() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glScalef(_zoom, _zoom, _zoom);
	if (_face_tracked) {
		glColor3f(0.0, 1.0, 0.0);
	}
	else {
		glColor3f(1.0, 0.0, 0.0);
	}
	glPointSize(1.0f);
	glBegin(GL_POINTS);
	for (unsigned int i = 0; i < xyz_cloud->points.size(); ++i) {
		float x, y, z;
		x = xyz_cloud->points[i].x;
		y = xyz_cloud->points[i].y;
		z = xyz_cloud->points[i].z;
		glVertex3f(x,y,z);
	}
	glEnd();

	if (_face_tracked) {
		glColor3f(0.0, 1.0, 0.0);
	}
	else {
		glColor3f(1.0, 0.0, 0.0);
	}
	glPopMatrix();
}

void GLWidget::setZoomValue(int val) {
	_zoom = (float)val / 10;
}

QString GLWidget::getMode() {
	return _mode;
}
void GLWidget::paintGL()
{
	if (_mode == "RGB") {
		paintRGB();
	}
	else if (_mode == "PCL") {
		paintPCL();
	}
	
}

void GLWidget::resizeGL(int width, int height)
{
	m_projection.setToIdentity();
	m_projection.perspective(60.0f, width / float(height), 0.01f, 1000.0f);

}
void GLWidget::mousePressEvent(QMouseEvent *event)
{

}
void GLWidget::mouseMoveEvent(QMouseEvent *event)
{

}

void GLWidget::setFaceTracked(bool tracked) {
	_face_tracked = tracked;
}

void GLWidget::setCurrentFrameToShow(QImage &frame) {
	//texture = frame;
	gldata = QGLWidget::convertToGLFormat(frame);
	
	//gldata = QGLWidget::convertToGLFormat(*texture);
}

void GLWidget::setCurrentFramePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud) {
	this->xyz_cloud = xyz_cloud;
}

void GLWidget::setCurrentFrameRectangle(PXCPoint3DF32 rectangle[4]) {
	_rectangle[0] = rectangle[0];
	_rectangle[1] = rectangle[1];
	_rectangle[2] = rectangle[2];
	_rectangle[3] = rectangle[3];
}

void GLWidget::setCurrentFrameRectangleRGB(PXCPointF32 rectangle[4]) {
	_rectangleRGB[0] = rectangle[0];
	_rectangleRGB[1] = rectangle[1];
	_rectangleRGB[2] = rectangle[2];
	_rectangleRGB[3] = rectangle[3];
}
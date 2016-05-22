#include "GLWidget.h"
#include <QMouseEvent>
#include <QOpenGLShaderProgram>
#include <QCoreApplication>
#include <math.h>
#include <QGLWidget>

#define TIME 10
GLWidget::GLWidget(QWidget *parent)
	: QOpenGLWidget(parent)
{
	//Add things here
	QSurfaceFormat format;
	format.setDepthBufferSize(24);
	setFormat(format);
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(update()));
	timer->start(10);

	timer->start(TIME);
	_face_tracked = false;
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

void GLWidget::initTexture(QImage texture) {
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
void GLWidget::paintGL()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
	glTexCoord2f(1, 0); glVertex3f(-1, -1, -1);
	glTexCoord2f(0, 0); glVertex3f(1, -1, -1);
	glTexCoord2f(0, 1); glVertex3f(1, 1, -1);
	glTexCoord2f(1, 1); glVertex3f(-1, 1, -1);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	if (_face_tracked) {
		glColor3f(0.0, 1.0, 0.0);
	}
	else {
		glColor3f(1.0, 0.0, 0.0);
	}
	glBegin(GL_QUADS);
	glVertex3f(-1, -1, -1);
	glVertex3f(-1+0.05, -1, -1);
	glVertex3f(-1+0.05, -1+0.05, -1);
	glVertex3f(-1, -1+0.05, -1);
	glEnd();
	glColor3f(1.0,1.0, 1.0);
	/*
	glTexCoord2d(0, 0);// glVertex2d(0, 0);
	glTexCoord2d(0.6, 0);// glVertex2d(gldata.width(), 0);
	glTexCoord2d(0.6, 0.6);// glVertex2d(gldata.width(), gldata.height());
	glTexCoord2d(0, 0.6);// glVertex2d(0, gldata.height());
	glEnd();
	
	*/
	
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
	texture = frame;
	gldata = QGLWidget::convertToGLFormat(texture);
	
	//gldata = QGLWidget::convertToGLFormat(*texture);
}
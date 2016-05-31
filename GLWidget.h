#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <qopengltexture.h>
#include <qtimer.h>
#include <qpainter.h>
#include <qmatrix4x4.h>
class GLWidget :
	public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();
	void clean();
	void setCurrentFrameToShow(QImage &frame);
	void initTexture(QImage texture);
	void setFaceTracked(bool tracked);
	void changeMode(QString mode);
protected:
	void initializeGL() Q_DECL_OVERRIDE;
	void paintGL() Q_DECL_OVERRIDE;
	void resizeGL(int width, int height) Q_DECL_OVERRIDE;
	void mousePressEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	void mouseMoveEvent(QMouseEvent *event) Q_DECL_OVERRIDE;
	
	

private:
	QImage texture;
	QImage gldata;
	QPainter painter;
	GLuint textureID;
	QMatrix4x4 m_projection;
	bool _face_tracked;
	QString _mode;

	void paintRGB();
	void paintPCL();
};


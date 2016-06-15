#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <qopengltexture.h>
#include <qtimer.h>
#include <qpainter.h>
#include <qmatrix4x4.h>
#include "pxcsensemanager.h"
#include "pxcmetadata.h"
#include "pxcprojection.h"
#undef max
#undef min
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
typedef pcl::PointCloud<pcl::PointXYZ> CloudType;
class GLWidget :
	public QOpenGLWidget, protected QOpenGLFunctions
{
	Q_OBJECT

public:
	GLWidget(QWidget *parent = 0);
	~GLWidget();
	void clean();
	void setCurrentFrameToShow(QImage &frame);
	void setCurrentFramePCL(pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud);
	void setCurrentFrameRectangle(PXCPoint3DF32 rectangle[4]);
	void setCurrentFrameRectangleRGB(PXCPointF32 rectangle[4]);
	void initTexture(const QImage& texture);
	void setFaceTracked(bool tracked);
	void changeMode(QString mode);
	QString getMode();
public slots:
	void setZoomValue(int val);
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

	float _zoom;
	PXCPoint3DF32 _rectangle[4];
	PXCPointF32 _rectangleRGB[4];
	CloudType::Ptr xyz_cloud;

	void paintRGB();
	void paintPCL();
};


#pragma once
#define _CRT_SECURE_NO_DEPRECATE


//
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "cinder/MayaCamUI.h"
#include "cinder/app/AppBasic.h"
#include "cinder/params/params.h"
#include "Kinect2.h"
#include "cinder/gl/Texture.h"
#include "cinder/gl/Vbo.h"



// We'll create a new Cinder Application by deriving from the AppBasic class
class BasicApp : public ci::app::AppBasic {
public:
	void mouseDown(ci::app::MouseEvent event);
	void mouseDrag(ci::app::MouseEvent event);
	void keyDown(ci::app::KeyEvent event);
	void draw();
	void setup();
	void update();


	// This will maintain a list of points which we will draw line segments between
	std::list<ci::Vec2f>		mPoints;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;

	ci::MayaCamUI mMayaCam;


private:
	Kinect2::DeviceRef	mKinect;
	Kinect2::Frame		mFrame;

	ci::gl::VboMesh			mVboMesh;

	ci::params::InterfaceGlRef mParams;
	int mMeanK;
	float mThresh;
	float mFrameRate;


	void createVbo(ci::Channel8u ch);
};
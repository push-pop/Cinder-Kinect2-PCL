#include "Cinder-PCL.h"
#include <list>


using namespace ci;
using namespace ci::app;
using namespace std;


void kinectToPcl(const Kinect2::Frame& frame, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//TODO
	
}

void pclToKinect(Kinect2::Frame& frame, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	//TODO
}

void pclToSurface(ci::Surface& surface, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

	//TODO
}

void BasicApp::mouseDown(MouseEvent event)
{
	mMayaCam.mouseDown(event.getPos());
}

void BasicApp::mouseDrag(MouseEvent event)
{
	mMayaCam.mouseDrag(event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown());
}

void BasicApp::keyDown(KeyEvent event)
{
	if (event.getChar() == 'f')
		setFullScreen(!isFullScreen());
}

void BasicApp::draw()
{

	gl::clear(Color(0.1f, 0.1f, 0.15f));

	gl::color(1.0f, 1.0f, 1.0f);

	if (mFrame.getDepth())
	{
		gl::TextureRef tex = ci::gl::Texture::create(Kinect2::channel16To8(mFrame.getDepth()));
		gl::draw(tex, tex->getBounds(), ci::Rectf(Vec2f(0, 0), Vec2f(512, 424)));
	}

	

	mParams->draw();
}

void BasicApp::update()
{

	mFrameRate = getAverageFps();

	if (mKinect && mKinect->getFrame().getTimeStamp() > mFrame.getTimeStamp()){
		mFrame = mKinect->getFrame();
	}
	
	kinectToPcl((mKinect->getFrame()), cloud);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(cloud);
	sor.setMeanK(mMeanK);
	sor.setStddevMulThresh(mThresh);
	sor.filter(*cloud_filtered);


	console() << "Cloud before filtering: " << std::endl;
	console() << *cloud << std::endl;

	console() << "Cloud after filtering: " << std::endl;
	console() << *cloud_filtered << std::endl;
}

void BasicApp::setup()
{

	CameraPersp initialCam;
	initialCam.setPerspective(45.0f, getWindowAspectRatio(), 0.1, 10000);
	mMayaCam.setCurrentCam(initialCam);

	mKinect = Kinect2::Device::create();
	mKinect->start(Kinect2::DeviceOptions().enableInfrared().enableDepth());

	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	cloud_filtered = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PCDReader reader;
	reader.read<pcl::PointXYZ>("table_scene_lms400.pcd", *cloud);
	mMeanK = 50;
	mThresh = 1.0;
	mFrameRate = 0.0;

	mParams = params::InterfaceGl::create("Params", Vec2i(200, 100));
	mParams->addParam("Frame rate", &mFrameRate, "", true);
	mParams->addParam("Mean K", &mMeanK, "min=0, max=50, step=1");
	mParams->addParam("Threshold", &mThresh, "min=0, max=1.0, step=0.01");
	mParams->addButton("Quit", bind(&BasicApp::quit, this), "key=q");


}

void BasicApp::createVbo(ci::Channel8u ch)
{
	gl::VboMesh::Layout layout;

	layout.setStaticPositions();
	layout.setStaticTexCoords2d();
	layout.setStaticIndices();

	int numVertices = ch.getWidth()*ch.getHeight();
	int numShapes = (ch.getWidth() - 1)*(ch.getHeight() - 1);

	mVboMesh = gl::VboMesh(numVertices, numShapes, layout, GL_POINTS);
}



// This line tells Cinder to actually create the application
CINDER_APP_BASIC(BasicApp, RendererGl)
/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-06-04 09:05
#
# Filename:		main.cpp
#
# Description: 
#
===============================================*/

#include "systems/plane_edge_SLAM.h"

using namespace Eigen;
using namespace std;
using namespace ulysses;

bool stop;
void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
	pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
	if (event.getKeySym () == "q" && event.keyDown ())
	{
	   viewer->close();
	}
	if (event.getKeySym () == "c" && event.keyDown ())
	{
		stop=true;
	}
}

int main(int argc, char *argv[])
{
	stop=false;
	double color_r,color_g,color_b;

//	bool debug;
	std::string mode;

	std::string settingFile="run/settings.yaml";
	cv::FileStorage settings(settingFile,cv::FileStorage::READ);
	settings["mode"]>>mode;
	settings["BackgroundColor.r"]>>color_r;
	settings["BackgroundColor.g"]>>color_g;
	settings["BackgroundColor.b"]>>color_b;
	settings.release();

		float radius = 7.0f;
		float theta = M_PI*11.0/12.0;//0.0f;
		float phi = 0.0f;
	vtkObject::GlobalWarningDisplayOff();
	boost::shared_ptr<pcl::visualization::PCLVisualizer> vis(new pcl::visualization::PCLVisualizer ("Edge-Plane-SLAM"));
	vis->setBackgroundColor (color_r,color_g,color_b);
	vis->initCameraParameters ();
	vis->registerKeyboardCallback (keyboardEventOccurred, (void*)vis.get());
	vis->setCameraPosition(0.0336763, -17.3928, 2.40394, 0.999996, 0.00176617, -0.00210663);
	vis->spinOnce(1, true);

//		vis->setCameraPosition(radius * sin(theta) * cos(phi),
//									  radius * sin(theta) * sin(phi),
//									  radius * cos(theta),
//									  0,
//									  1,
//									  0);
//		vis->setSize(1180, 850);
		vis->setSize(1080, 850);
//		Eigen::Affine3f coord(ulysses::Transform::Identity().getMatrix4f());
//		vis->addCoordinateSystem(1.0,coord,"coord");

	ulysses::PlaneEdgeSLAM system(settingFile);

//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::io::loadPCDFile<pcl::PointXYZ>("living-room.pcd",*cloud);
//	vis->addPointCloud(cloud,"lr");
//	vis->spin();
//	return 0;

//	if(mode=="test") 
//	system.planeFitting(vis);
//	system.trajGeneration(vis);
	if(mode=="track") system.track(vis);
	if(mode=="traj") system.trajPuzzle(vis);
	if(mode=="record") system.videoRecording(vis);
	if(mode=="recordPlane") system.videoRecordingPlane(vis);
	if(mode=="assess") system.assessCloud(vis);

	return 0;

}


/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-06-09 09:10
#
# Filename:		plane_edge_SLAM.h
#
# Description: 
#
************************************************/
#ifndef _PLANE_EDGE_SLAM_H_
#define _PLANE_EDGE_SLAM_H_

#include "types/types.h"
#include "types/map.h"
#include "features/plane_segmentation.h"
#include "features/plane_extraction.h"
#include "features/plane_fitting.h"
#include "trajectory/traj_puzzle.h"
#include "features/it_geometric_feature_matching.h"
#include "features/edge_extraction.h"
//#include "features/line_extraction.h"
//#include "features/geometric_feature_matching.h"
#include "motion/motion_estimation.h"
#include <sys/time.h>
#include <pcl-1.8/pcl/io/pcd_io.h>

#include "systems/ThreadMutexObject.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/icp.h>
#include "ThreadMutexObject.h"

extern bool stop;
namespace ulysses
{
	extern double THRES_RAD, THRES_DIST;
	extern CameraIntrinsic camera_intrinsic;

	class PlaneEdgeSLAM
	{
	public:
		PlaneEdgeSLAM(const std::string &settingFile);
		~PlaneEdgeSLAM()
		{
//			delete scan_cur;
			delete map;
//			delete feature_matching;
			delete plane_segmentation;
			delete plane_extraction;
			delete plane_fitting;
			delete traj_puzzle;
			delete it_gfm;
			delete edge_extraction;
//			delete line_extraction;
			delete motion_estimation;
		}

//		Scan* getCurrentScan() const {return scan_cur;}
//		Map* getMap() const {return map;}
//		double time(int i) const {return timestamps[i];}
//		int size() const {return timestamps.size();}

		double planeFitting(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double trajGeneration(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double track(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double trajPuzzle(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double videoRecording(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double videoRecordingPlane(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
		double assessCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);


//		double simulate(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//
//		double optimize(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//		double optimize_map(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//
//		double associate(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//		double localize(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);
//
//		void visScan(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);


	private:

		std::string mode;
		std::string plane_fitting_method;

		bool debug;
		double debug_time;
		double start_time, end_time, delta_time;
		int vis_scans;
		bool save_cloud;

		std::ofstream fp;
		std::string save_folder;
		std::string load_folder;
		std::string seq_folder;
		double current_time;
//		bool isFirstFrame;

		bool use_gt;

		Scan *scan_cur, *scan_ref;
		Map *map;


		PlaneSegmentation *plane_segmentation;
		PlaneExtraction *plane_extraction;
		PlaneFitting *plane_fitting;
		TrajPuzzle *traj_puzzle;
		BaselessCliff::GeometricFeatureMatching *it_gfm;
		EdgeExtraction *edge_extraction;
//		LineExtraction *line_extraction;
//		GeometricFeatureMatching *feature_matching;
		MotionEstimation *motion_estimation;
//		GlobalMap *global_map;
//		RelativePoseError *rpe;

		std::vector<double> timestamps;
		std::map<double,std::string> files_depth;
		std::map<double,std::string> files_rgb;
		std::map<double,Transform> Tcw_truth;
		void loadImages(const std::string &folder);

		std::vector<double> timestamps_traj;
		std::map<double,Transform> Tcg_traj;
		void loadTraj(const std::string &folder);

		void saveMap() const 
		{
			if(!debug)
			{
				map->save(save_folder);
				map->saveTimes(save_folder); // filenames in /scans folder;
			}
		}



//		void constraints(Scan *scan)
//		{
//			const int N=scan->sizeFeature();
//			Eigen::MatrixXd A=Eigen::MatrixXd::Zero(N*3,3);
//			int i=0;
//			for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++,i++)
//			{
//				Feature *f=it->second;
//				if(f->Type()==PLANE)
//				{
//					A.block<1,3>(i*3,0)=f->getDirection().transpose();
//				}
//				else if(f->Type()==LINE)
//				{
//					A.block<3,3>(i*3,0)=ulysses::Transform::skew_sym(f->getDirection());
//				}
//			}
//			Eigen::Matrix3d ATA=A.transpose()*A;
//
//			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(ATA);
//			Eigen::Vector3d eigenvalues=es.eigenvalues();
////			Eigen::Matrix3d eigenvectors=es.eigenvectors();
////			fp<<"eigenvalues="<<eigenvalues.transpose()<<std::endl;
////			fp<<"eigenvectors="<<std::endl<<eigenvectors<<std::endl<<std::endl;
//			
//			scan->Constraints()=eigenvalues;
//		}
//
//		void saveConstraints()
//		{
//			fp.open(save_folder+"/constraints.txt");
//			const_iterScan it_scan=map->beginScan();
//			const_iterCamera it_cam=map->beginCamera();
//			Transform Tgw=it_cam->second.inv()*it_scan->second->Tcw();
//			for(const_iterScan it=map->beginScan();it!=map->endScan()&&it_cam!=map->endCamera();it++,it_cam++)
//			{
//				constraints(it->second);
//				Transform Tres=it_cam->second.inv()*it->second->Tcw()*Tgw.inv();
//				Vector6d xi=Tres.getMotionVector();
//				double ratio=it->second->Constraints()(2)/it->second->Constraints()(0);
//				fp<<std::fixed<<it->second->time()<<" "
//				  <<xi.block<3,1>(0,0).norm()<<" "<<xi.block<3,1>(3,0).norm()<<" "
//				  <<it->second->Constraints().transpose()<<" "<<ratio<<std::endl;
//			}
//			fp.close();
//		}
	};
}

#endif

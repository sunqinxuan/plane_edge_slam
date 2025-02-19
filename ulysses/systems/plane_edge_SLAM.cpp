/***********************************************
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-06-09 09:10
#
# Filename:		plane_edge_SLAM.cpp
#
# Description: 
#
************************************************/
#include "systems/plane_edge_SLAM.h"

namespace ulysses 
{
//	CameraIntrinsic camera_intrinsic;
	double THRES_RAD, THRES_DIST;

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr rCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr mCloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		ThreadMutexObject<bool> done(false);
		ThreadMutexObject<int> count(0);
		ThreadMutexObject<int> iteration(0);
		std::string plyFile;
//		double sleep_time;

		float getScore(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input);
		void computeAlignment();
		void compareCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	void vis_addCamera(Transform Tgc, double time, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		// vis->removeAllPointClouds();
		// vis->removeAllShapes();
		char id[20];
		pcl::PointXYZRGBA pt1,pt2;
		double scale=0.3;
		// for(size_t i=0;i<map->seq.size();i++)
		{
//		Transform Tgc=scan->Tcg.inv();
		Eigen::Vector3d x=Tgc.R.block<3,1>(0,0);
		Eigen::Vector3d y=Tgc.R.block<3,1>(0,1);
		Eigen::Vector3d z=Tgc.R.block<3,1>(0,2);
		pt1.x=Tgc.t(0);
		pt1.y=Tgc.t(1);
		pt1.z=Tgc.t(2);
		// x - green
		pt2.x=pt1.x+x(0)*scale;
		pt2.y=pt1.y+x(1)*scale;
		pt2.z=pt1.z+x(2)*scale;
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,255,0,std::to_string(time)+"xaxis");
		// y - blue
		pt2.x=pt1.x+y(0)*scale;
		pt2.y=pt1.y+y(1)*scale;
		pt2.z=pt1.z+y(2)*scale;
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,0,0,255,std::to_string(time)+"yaxis");
		// z - red
		pt2.x=pt1.x+z(0)*scale;
		pt2.y=pt1.y+z(1)*scale;
		pt2.z=pt1.z+z(2)*scale;
		vis->addLine<pcl::PointXYZRGBA>(pt1,pt2,255,0,0,std::to_string(time)+"zaxis");
		}
	}

	PlaneEdgeSLAM::PlaneEdgeSLAM(const std::string &settingFile)
	{
		remove("PlaneEdgeSLAM.txt");

		cv::FileStorage settings(settingFile,cv::FileStorage::READ);
		settings["debug"]>>debug;
		settings["mode"]>>mode;
		settings["PlaneFitting.method"]>>plane_fitting_method;
		settings["debug_time"]>>debug_time;
		settings["start_time"]>>start_time;
		settings["end_time"]>>end_time;
		settings["delta_time"]>>delta_time;
		settings["vis_scans"]>>vis_scans;
		settings["save_cloud"]>>save_cloud;
		settings["loadFolder"]>>load_folder;
		settings["seqFolder"]>>seq_folder;
		settings["useGT"]>>use_gt;
		settings["THRES_RAD"]>>THRES_RAD; THRES_RAD*=(M_PI/180.0);
		settings["THRES_DIST"]>>THRES_DIST;
		settings["Camera.fx"]>>camera_intrinsic.fx;
		settings["Camera.fy"]>>camera_intrinsic.fy;
		settings["Camera.cx"]>>camera_intrinsic.cx;
		settings["Camera.cy"]>>camera_intrinsic.cy;
		settings["Camera.width"]>>camera_intrinsic.width;
		settings["Camera.height"]>>camera_intrinsic.height;
		settings["Camera.factor"]>>camera_intrinsic.factor;
		settings["Camera.m_fp"]>>camera_intrinsic.m_fp;
		settings["Camera.sigma_u"]>>camera_intrinsic.sigma_u;
		settings["Camera.sigma_v"]>>camera_intrinsic.sigma_v;
		settings["Camera.sigma_disparity"]>>camera_intrinsic.sigma_disparity;

		settings["plyFile"]>>plyFile;
//		settings["sleepTime"]>>sleep_time;
		settings.release();

		loadImages(seq_folder);

//		if(plane_method==1) plane_extraction=new PlaneExtraction(settingFile);
//		else if(plane_method==2) 
		plane_segmentation=new PlaneSegmentation(settingFile);
		plane_extraction=new PlaneExtraction(settingFile);
		plane_fitting=new PlaneFitting(settingFile);
		traj_puzzle=new TrajPuzzle();
//		
//		line_extraction=new LineExtraction(settingFile);
//		feature_matching=new GeometricFeatureMatching(settingFile);
		motion_estimation=new MotionEstimation(settingFile);
		it_gfm=new BaselessCliff::GeometricFeatureMatching(settingFile);
		edge_extraction=new EdgeExtraction(settingFile);

//		global_map=new GlobalMap(settingFile);
//		global_map->setDebug(debug);
		scan_cur=0; scan_ref=0;
		map=new Map;
//		map_true=new Map;
//		map_true->load(load_folder);

		timeval time;
		gettimeofday(&time,NULL);
		current_time=time.tv_sec+time.tv_usec*1e-6;

		if(!debug)
		{
			save_folder="slam_"+std::to_string(current_time);
			std::string mkdir_scans="mkdir -p "+save_folder+"/scans";
			if(system(mkdir_scans.c_str()));
		}
	}

	double PlaneEdgeSLAM::planeFitting(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
//		if(debug)
		{
			fp.open("PlaneEdgeSLAM.txt",std::ios::app);
		}

		double last_time=0;
		std::ofstream fp_out;
		std::ifstream fp_in;
		std::string filename=load_folder+"/"+plane_fitting_method+".txt";

		if(mode=="load") fp_out.open(filename,std::ios::out);
		if(mode=="evaluate") fp_in.open(filename,std::ios::in);


		for(int i=0;i<timestamps.size();i++)
		{
			if(stop) break;
			if(debug) if(timestamps[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(timestamps[i]-start_time<-1e-4) continue;
				if(timestamps[i]>end_time) continue;
			}
			if(timestamps[i]-last_time<delta_time) continue;
			last_time=timestamps[i];

			scan_cur=new Scan(timestamps[i]);
			scan_cur->loadScan(timestamps[i],files_depth.find(timestamps[i])->second,files_rgb.find(timestamps[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps[i])->second;

			if(debug) fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
			std::cout<<std::endl;
			std::cout<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;

			// save segmented planes from un-noisy surfaces;
			// seqFolder = "living_room_traj0_frei_png"
			if(mode=="save")
			{
				plane_segmentation->extractPlanes(scan_cur);
				plane_fitting->fitPlanes(scan_cur);
				std::cout<<"extracted "<<scan_cur->sizeFeature()<<" features in current scan"<<std::endl;
				if(debug) {fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);}
				else {scan_cur->saveFeatures(save_folder);}
			}

			// load segmented planes and substitute noisy points;
			// fit plane models using either LS or PR;
			// seqFolder = "living_room_traj0n_frei_png"
			if(mode=="load")
			{
				if(!scan_cur->loadFeatures(load_folder)) break;
				for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
				{
					if(it->second->Type()==PLANE)
					{
						Eigen::Vector3d n=it->second->plane()->n;
						double d=it->second->plane()->d;
						plane_fitting->removeOutliers(it->second->plane());
						plane_fitting->fitPlaneModel(it->second->plane());
						fp_out<<it->second<<endl;//noisy 
					}
				}
			}

			// load the un-noisy points and computes average distance 
			// from the original points to the fitted planes 
			// stored in files "LS.txt" and "PR.txt";
			// seqFolder = "living_room_traj0_frei_png"
			if(mode=="evaluate")
			{
				if(!scan_cur->loadFeatures(load_folder)) break;
				std::stringstream ss;
				for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
				{
					if(it->second->Type()==PLANE)
					{
						std::string s, id;
						Eigen::Vector3d n; // n=plane->n;
						double d; // d=plane->d;
						ptrPointCloud points=scan_cur->points();
						Plane *plane=it->second->plane();
						double dist;

//						std::cout<<std::endl;

						// LS
						getline(fp_in,s);
						if(!s.empty())
						{
							ss.clear(); ss<<s; 
							ss>>id;
							ss>>n(0)>>n(1)>>n(2)>>d;
							dist=0;
							for(int i=0;i<plane->indices.size();i++)
							{
								int idx=plane->indices[i];
								Eigen::Vector3d p;
								p(0)=points->at(idx).x;
								p(1)=points->at(idx).y;
								p(2)=points->at(idx).z;
								dist+=fabs(n.dot(p)+d);
							}
							dist/=plane->indices.size();
//							std::cout<<"\t"<<dist<<"\t"<<asin(n.cross(plane->n).norm())*180.0/M_PI<<"\t"<<fabs(d-plane->d)<<std::endl;
							fp<<std::fixed<<scan_cur->time()<<"\t"<<dist<<"\t"
							  <<asin(n.cross(plane->n).norm())*180.0/M_PI<<"\t"<<fabs(d-plane->d)<<endl;
						}
					}
				}
			}

//			scan_cur->visScan(vis);

			delete scan_cur;
		}
		if(mode=="load") fp_out.close();
		if(mode=="evaluate") fp_in.close();

		if(debug) fp.close();
		return 0;
	}

	double PlaneEdgeSLAM::trajGeneration(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("PlaneEdgeSLAM.txt",std::ios::app);
		}

		double last_time=-1;

		for(int i=0;i<timestamps.size();i++)
		{
			if(stop) break;
			if(debug) if(timestamps[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(timestamps[i]-start_time<-1e-4) continue;
				if(timestamps[i]>end_time) continue;
			}
			if(timestamps[i]-last_time<delta_time) continue;

			scan_cur=new Scan(timestamps[i]);
			scan_cur->loadScan(timestamps[i],files_depth.find(timestamps[i])->second,files_rgb.find(timestamps[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps[i])->second;

			std::cout<<std::endl;
			std::cout<<"scan.timestamp = "<<std::fixed<<timestamps[i]<<std::endl;

//			plane_segmentation->extractPlanes(scan_cur);
			plane_extraction->extractPlanes(scan_cur,vis);
			plane_fitting->fitPlanes(scan_cur);
			std::cout<<"extracted "<<scan_cur->sizeFeature()<<" features in current scan"<<std::endl;

			if(debug)
			{
				vis->removeAllPointClouds();
				scan_cur->visScanFeatures(vis,IDENTITY);
				vis->spin();
			}

			Eigen::Matrix3d Q=Eigen::Matrix3d::Zero();
			
			for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
			{
				if(it->second->Type()==PLANE)
				{
					std::string s, id;
					Eigen::Vector3d n; // n=plane->n;
					Plane *plane=it->second->plane();
					n=plane->n;
					Q+=n*n.transpose();
				}
			}

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
			es.compute(Q);
			Eigen::Vector3d eigenvalues=es.eigenvalues();
			Eigen::Matrix3d eigenvectors=es.eigenvectors();
			eigenvalues=eigenvalues.cwiseAbs();
			double min=DBL_MAX, max=DBL_MIN;
			int idx_min=-1, idx_max=-1;
			for(int i=0;i<3;i++)
			{
				if(eigenvalues(i)<min)
				{
					min=eigenvalues(i);
					idx_min=i;
				}
				if(eigenvalues(i)<max)
				{
					max=eigenvalues(i);
					idx_max=i;
				}
			}
			Eigen::Vector3d axis_max=eigenvectors.block<3,1>(0,idx_max);
			Eigen::Vector3d axis_min=eigenvectors.block<3,1>(0,idx_min);

			Eigen::Vector3d e(1.0,1.0,1.0);
			if(axis_min.dot(e)<0) axis_min=-axis_min;

//			axis_max(0)=1.0;
//			axis_max(1)=0.0;
//			axis_max(2)=1.0;
//			axis_max.normalize();
//			axis_min=axis_max;

			if(last_time>0)
			{
				scan_cur->ref()=scan_ref;

				delete scan_ref;
			}

			scan_cur->Tcg()=Tcw_truth.find(timestamps[i])->second;
			scan_ref=scan_cur;
			last_time=timestamps[i];

		}

		if(debug) fp.close();
		return 0;
	}

	using namespace std;
	double PlaneEdgeSLAM::track(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("PlaneEdgeSLAM.txt",std::ios::app);
		}

		double last_time=-1;
		std::ofstream fp_traj;
		fp_traj.open("traj_"+std::to_string(start_time)+".txt",std::ios::out);
		int count=0;

		for(int i=0;i<timestamps.size();i++)
		{
			if(stop) break;
			if(debug) if(timestamps[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(start_time!=0)
				{
					if(timestamps[i]<start_time) continue;
				}
				if(end_time!=0)
				{
					if(timestamps[i]>end_time) break;
				}
			}
			if(timestamps[i]-last_time<delta_time) continue;

			count++;
			scan_cur=new Scan(timestamps[i]);
//			cout<<fixed<<timestamps[i]<<endl;
//			cout<<files_depth.find(timestamps[i])->second<<endl;
//			cout<<files_rgb.find(timestamps[i])->second<<endl;
			scan_cur->loadScan(timestamps[i],files_depth.find(timestamps[i])->second,files_rgb.find(timestamps[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps[i])->second;

//			if(debug) fp<<"scan.timestamp = "<<std::fixed<<scan_cur->time()<<std::endl;
			std::cout<<std::endl;
			std::cout<<"scan.timestamp = "<<std::fixed<<timestamps[i]<<std::endl;

//			plane_segmentation->extractPlanes(scan_cur);
			plane_extraction->extractPlanes(scan_cur,vis);
			plane_fitting->fitPlanes(scan_cur);
			std::cout<<"extracted "<<scan_cur->sizeFeature()<<" features in current scan"<<std::endl;
			if(debug) {fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);}

			edge_extraction->extract(scan_cur);
//			edge_extraction->vis(scan_cur,vis);

			if(debug) { scan_cur->visScanFeatures(vis); vis->spin(); }

			if(scan_ref!=0)
			{
				scan_cur->ref()=scan_ref;

				it_gfm->match(scan_cur,vis);
				if(debug) {fp<<"extracted features: "<<std::endl; scan_cur->printFeatures(fp);}
//				scan_cur->association()->vis(scan_cur,vis);

//				map->clear();
//				map->addScan(scan_cur);
				motion_estimation->estimate(scan_cur,vis);
				if(use_gt)
				{
					scan_cur->Tcr()=scan_cur->Tcw()*scan_cur->ref()->Tcw().inv();
					scan_cur->localize();
				}

				if(!debug) 
				{
					std::cout<<"Tcr = "<<scan_cur->Tcr()<<std::endl;
					scan_cur->vis2Scans(vis,IDENTITY);
					scan_cur->vis2Scans(vis,ESTIMATE);
				}
//				if(!debug)
//				{
//					scan_cur->vis(vis,ESTIMATE);
//					if(count%vis_scans==0) 
//					{
//						vis->spin();
//						vis->removeAllPointClouds();
//					}
//				}

				delete scan_ref;
			}
			fp_traj<<std::fixed<<timestamps[i]<<" "<<scan_cur->Tcg().inv()<<std::endl;

			scan_ref=scan_cur;
			last_time=timestamps[i];
		}

//		vis->spin();

		if(debug) fp.close();
		fp_traj.close();
		return 0;
	}

	double PlaneEdgeSLAM::trajPuzzle(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		traj_puzzle->readTrajFiles(load_folder);
		return 0;
	}


	float getScore(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input)
	{
		pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
		tree->setInputCloud(mCloud);

		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		double totalSum = 0;

		count.assignValue(0);

		for(size_t i = 0; i < input->size(); i++)
		{
			tree->nearestKSearch(input->at(i), 1, pointIdxNKNSearch, pointNKNSquaredDistance);
			totalSum += sqrt(pointNKNSquaredDistance.at(0));
			count++;
		}

		return totalSum / (double)input->size();
	}

	void computeAlignment()
	{
		float value = getScore(rCloud);

//		if(value < 0.05)
		{
			pcl::IterativeClosestPoint<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;

			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr aligned (new pcl::PointCloud <pcl::PointXYZRGBNormal>);

			icp.setInputSource(rCloud);
			icp.setInputTarget(mCloud);
			icp.setMaximumIterations(1);

			for(int i = 0; i < 10; i++)
			{
				icp.align(*aligned, icp.getFinalTransformation());
				iteration++;
			}

			value = std::min(getScore(aligned), value);
		}

		std::cout << value << std::endl;

		done.assignValue(true);
	}

	void compareCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		vis->removeAllPointClouds();
		vis->removeAllCoordinateSystems();
//		pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
//
//		std::string reconstructionFile;
//		pcl::console::parse_argument(argc, argv, "-r", reconstructionFile);
//
//		std::string modelFile;
//		pcl::console::parse_argument(argc, argv, "-m", modelFile);
//
//		if(reconstructionFile.length() == 0 || modelFile.length() == 0)
//		{
//			std::cout << "Please provide input files with -r and -m" << std::endl;
//			exit(1);
//		}

		pcl::PCLPointCloud2 rPoints;

		pcl::toPCLPointCloud2(*rCloud, rPoints);

		for(int i = 0; i < rPoints.fields.size(); i++)
		{
			if(rPoints.fields.at(i).name.compare("curvature") == 0)
			{
				rPoints.fields.at(i).name = "radius";
			}
		}

		pcl::io::loadPLYFile(plyFile, rPoints);

		pcl::fromPCLPointCloud2(rPoints, *rCloud);

		pcl::PCLPointCloud2 mPoints;

		pcl::toPCLPointCloud2(*mCloud, mPoints);

		for(int i = 0; i < mPoints.fields.size(); i++)
		{
			if(mPoints.fields.at(i).name.compare("curvature") == 0)
			{
				mPoints.fields.erase(mPoints.fields.begin() + i);
				i--;
			}
		}

//		pcl::io::loadPLYFile("living-room.ply", mPoints);
//
//		pcl::fromPCLPointCloud2(mPoints, *mCloud);
//
////		int trajectory = 0;
////		pcl::console::parse_argument(argc, argv, "-t", trajectory);
//
//		Eigen::Matrix4f trans;
//		trans << 0.999759, -0.000287637, 0.0219655, -1.36022,
//				 0.000160294, 0.999983, 0.00579897, 1.48382,
//				 0.0219668, 0.00579404, -0.999742, 1.44256,
//				 0, 0, 0, 1;
//		trans.inverse();
//
//		pcl::transformPointCloud(*mCloud, *mCloud, trans);

		float radius = 7.0f;
		float theta = M_PI*11.0/12.0;//0.0f;
		float phi = 0.0f;

//		pcl::visualization::PCLVisualizer cloudViewer("SurfReg");
//		vis->setBackgroundColor(0, 0, 0);
//		vis->initCameraParameters();
//		vis->setCameraPosition(radius * sin(theta) * cos(phi),
//									  radius * sin(theta) * sin(phi),
//									  radius * cos(theta),
//									  0,
//									  1,
//									  0);
//		vis->setSize(1680, 1050);
//
//		if(pcl::console::find_argument(argc, argv, "-f") != -1)
//		{
//			vis->setFullScreen(true);
//		}

		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> color(rCloud);
		vis->addPointCloud<pcl::PointXYZRGBNormal>(rCloud, color, "Cloud");

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorM(1, 0, 0);
		vis->addPointCloud<pcl::PointXYZRGBNormal>(mCloud, colorM, "MCloud");
		vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "MCloud");

		boost::thread * computeThread = new boost::thread(computeAlignment);

//			cout<<"rCloud->size()="<<rCloud->size()<<endl;

		while(!done.getValue())
		{
			vis->spinOnce(1, true);
			vis->removeShape("text");

			int countVal = count.getValue();


			if(countVal == rCloud->size())
			{
				std::stringstream strs;

				strs << "Aligning... " << iteration.getValue() << "/" << 10;

				vis->addText(strs.str(), 20, 20, 50, 1, 0, 0, "text");

//				cout << "Aligning... " << iteration.getValue() << "/" << 10 <<endl;
//				cout << "---------- " << countVal << "/" << rCloud->size() <<endl;
			}
			else
			{
				std::stringstream strs;

				strs << "Scoring... " << countVal << "/" << rCloud->size();

				vis->addText(strs.str(), 20, 20, 50, 1, 0, 0, "text");

//				cout << "Scoring... " << countVal << "/" << rCloud->size() <<endl;
			}

			vis->setCameraPosition(radius * sin(theta) * cos(phi),
										  radius * sin(theta) * sin(phi),
										  radius * cos(theta),
										  0,
										  1,
										  0);

			theta += 0.015;
			phi -= 0.015;
//			usleep(sleep_time);
		}

		computeThread->join();

		delete computeThread;
	}

	double PlaneEdgeSLAM::assessCloud(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
//		if(debug)
//		{
//			fp.open("PlaneEdgeSLAM.txt",std::ios::app);
//		}

//		loadTraj(load_folder);

		pcl::VoxelGrid<pcl::PointXYZRGBNormal> vgm;

		Eigen::Matrix3d R;
        R << 0.999759, -0.000287637, 0.0219655,
             0.000160294, 0.999983, 0.00579897,
             0.0219668, 0.00579404, -0.999742; 
		Eigen::Vector3d t;
		t <<  -1.36022, 1.48382, 1.44256;
		Transform T(R,t);
		T.inverse();
		pcl::PCLPointCloud2 mPoints;
		pcl::toPCLPointCloud2(*mCloud, mPoints);
		for(int i = 0; i < mPoints.fields.size(); i++)
		{
			if(mPoints.fields.at(i).name.compare("curvature") == 0)
			{
				mPoints.fields.erase(mPoints.fields.begin() + i);
				i--;
			}
		}
		pcl::io::loadPLYFile("living-room.ply", mPoints);
		pcl::fromPCLPointCloud2(mPoints, *mCloud);
		pcl::transformPointCloud(*mCloud, *mCloud, T.getMatrix4f());
		vgm.setInputCloud(mCloud);
		vgm.setLeafSize(0.05f,0.05f,0.05f);
		vgm.filter(*mCloud);

//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorM(1, 0, 0);
//		vis->addPointCloud<pcl::PointXYZRGBNormal>(mCloud, colorM, "MCloud");
//		vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "MCloud");

		compareCloud(vis);

		return 0;
	}

	double PlaneEdgeSLAM::videoRecording(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("PlaneEdgeSLAM.txt",std::ios::app);
		}

		loadTraj(load_folder);

		/*
		pcl::VoxelGrid<pcl::PointXYZRGBNormal> vgm;

		Eigen::Matrix3d R;
        R << 0.999759, -0.000287637, 0.0219655,
             0.000160294, 0.999983, 0.00579897,
             0.0219668, 0.00579404, -0.999742; 
		Eigen::Vector3d t;
		t <<  -1.36022, 1.48382, 1.44256;
		Transform T(R,t);
		T.inverse();
		pcl::PCLPointCloud2 mPoints;
		pcl::toPCLPointCloud2(*mCloud, mPoints);
		for(int i = 0; i < mPoints.fields.size(); i++)
		{
			if(mPoints.fields.at(i).name.compare("curvature") == 0)
			{
				mPoints.fields.erase(mPoints.fields.begin() + i);
				i--;
			}
		}
		pcl::io::loadPLYFile("living-room.ply", mPoints);
		pcl::fromPCLPointCloud2(mPoints, *mCloud);
		pcl::transformPointCloud(*mCloud, *mCloud, T.getMatrix4f());
		vgm.setInputCloud(mCloud);
		vgm.setLeafSize(0.05f,0.05f,0.05f);
		vgm.filter(*mCloud);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorM(1, 0, 0);
		vis->addPointCloud<pcl::PointXYZRGBNormal>(mCloud, colorM, "MCloud");
		vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 1, "MCloud");
//		vis->spin();
		vis->spinOnce(2000);
		*/

		double last_time=-1, cur_time;

		timeval start, end;
		double timeused;
		std::ofstream fp_time;
		fp_time.open("time.txt",std::ios::out);
		cv::Mat img_depth,img_rgb;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> vg;

		Transform T0w;

		for(int i=0;i<timestamps_traj.size();i++)
		{
			if(stop) break;

			cur_time=timestamps_traj[i];
//			cout<<"cur_time="<<std::fixed<<cur_time<<endl;
			if(files_depth.find(cur_time)==files_depth.end()) continue;

			if(debug) if(timestamps_traj[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(start_time!=0)
				{
					if(timestamps_traj[i]<start_time) continue;
				}
				if(end_time!=0)
				{
					if(timestamps_traj[i]>end_time) break;
				}
			}

			if(last_time!=-1)
			{
				if(cur_time-last_time<delta_time) continue;
			}

			std::cout<<std::endl;
			std::cout<<"scan.timestamp = "<<std::fixed<<timestamps_traj[i]<<std::endl;

//			cout<<"last_time="<<std::fixed<<last_time<<endl;
			if(last_time!=-1)
			{
				gettimeofday(&start,NULL);
//				usleep((cur_time-last_time)*1e6*0.7);
				usleep(0.3*1e6);
				gettimeofday(&end,NULL);
				timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
				cout<<"last for "<<timeused<<" ms\t"<<endl;
			}

			img_depth=cv::imread(files_depth.find(timestamps_traj[i])->second,cv::IMREAD_UNCHANGED);
			img_rgb=cv::imread(files_rgb.find(timestamps_traj[i])->second,cv::IMREAD_UNCHANGED);
//			unsigned short max_depth=0;
			for(int r=0;r<img_depth.rows;r++)
			{
				for(int c=0;c<img_depth.cols;c++)
				{
					img_depth.at<unsigned short>(r,c)*=3;
//					if(max_depth<img_depth.at<unsigned short>(r,c))
//					{
//						max_depth=img_depth.at<unsigned short>(r,c);
//					}
				}
			}
//			cout<<"max_depth="<<max_depth<<endl;
			cv::imshow("rgb",img_rgb);
			cv::waitKey(1);
			cv::imshow("depth",img_depth);
			cv::waitKey(1);

			scan_cur=new Scan(timestamps_traj[i]);
			scan_cur->loadScan(timestamps_traj[i],files_depth.find(timestamps_traj[i])->second,files_rgb.find(timestamps_traj[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps_traj[i])->second;
			scan_cur->Tcg()=Tcg_traj.find(timestamps_traj[i])->second;

			if(last_time!=-1)
			{
				T0w=scan_cur->Tcw();
			}
			scan_cur->Tcw()=scan_cur->Tcw()*T0w.inv();

			if(save_cloud)
			{
				pcl::transformPointCloud(*scan_cur->points(),*cloud_scan,scan_cur->Tcg().inv().getMatrix4f());
				vg.setInputCloud(cloud_scan);
				vg.setLeafSize(0.01f,0.01f,0.01f);
				vg.filter(*cloud_filtered);
				*cloud=*cloud+*cloud_filtered;
			}

//			Eigen::Affine3f coords(scan_cur->Tcg().inv().getMatrix4f());
//			vis->addCoordinateSystem(0.1,coords,std::to_string(timestamps_traj[i])+"CoordsSys");
			vis_addCamera(scan_cur->Tcg().inv(),scan_cur->time(),vis);

			pcl::transformPointCloud(*scan_cur->points(),*cloud_scan,scan_cur->Tcg().inv().getMatrix4f());
			vg.setInputCloud(cloud_scan);
			vg.setLeafSize(0.01f,0.01f,0.01f);
			vg.filter(*cloud_filtered);

			if (!vis->updatePointCloud(cloud_filtered,std::to_string(scan_cur->time())))
				vis->addPointCloud(cloud_filtered,std::to_string(scan_cur->time()));
			vis->spinOnce();

			if(debug || last_time==-1) vis->spin();
//			if(debug) vis->spin();

			last_time=cur_time;
//			delete scan_cur;
		}

		vis->spin();

		if(save_cloud)
		{
			vg.setInputCloud(cloud);
			vg.setLeafSize(0.01f,0.01f,0.01f);
			vg.filter(*cloud);

			pcl::PCDWriter writer;
			writer.write<pcl::PointXYZRGBA>("pointcloud.pcd",*cloud,false);

			vis->removeAllPointClouds();
			vis->removeAllCoordinateSystems();
			vis->addPointCloud(cloud,"cloud");
			vis->spin();
		}

		if(debug) fp.close();
		fp_time.close();
		return 0;
	}

	double PlaneEdgeSLAM::videoRecordingPlane(boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		if(debug)
		{
			fp.open("PlaneEdgeSLAM.txt",std::ios::app);
		}

		loadTraj(load_folder);

//		Eigen::Matrix3d R;
//        R << 0.999759, -0.000287637, 0.0219655,
//             0.000160294, 0.999983, 0.00579897,
//             0.0219668, 0.00579404, -0.999742; 
//		Eigen::Vector3d t;
//		t <<  -1.36022, 1.48382, 1.44256;
//		Transform T(R,t);
//		T.inverse();
//		pcl::PCLPointCloud2 mPoints;
//		pcl::toPCLPointCloud2(*mCloud, mPoints);
//		for(int i = 0; i < mPoints.fields.size(); i++)
//		{
//			if(mPoints.fields.at(i).name.compare("curvature") == 0)
//			{
//				mPoints.fields.erase(mPoints.fields.begin() + i);
//				i--;
//			}
//		}
//		pcl::io::loadPLYFile("living-room.ply", mPoints);
//		pcl::fromPCLPointCloud2(mPoints, *mCloud);
//		pcl::transformPointCloud(*mCloud, *mCloud, T.getMatrix4f());
//		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> colorM(1, 0, 0);
//		vis->addPointCloud<pcl::PointXYZRGBNormal>(mCloud, colorM, "MCloud");
//		vis->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.1, "MCloud");
//		vis->spinOnce();

		double last_time=-1, cur_time;

		timeval start, end;
		double timeused;
		std::ofstream fp_time;
		fp_time.open("time.txt",std::ios::out);
		cv::Mat img_depth,img_rgb;
//		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);
		pcl::VoxelGrid<pcl::PointXYZRGBA> vg;

		float radius = 5.0f;
		float theta = M_PI*11.0/12.0;//0.0f;
		float phi = 0.0f;
		boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("Features"));
		viewer->setBackgroundColor (0,0,0);
		viewer->initCameraParameters ();
		viewer->spinOnce(1, true);
		viewer->setCameraPosition(radius * sin(theta) * cos(phi),
		 						  radius * sin(theta) * sin(phi),
		 						  radius * cos(theta),
		 						  0, 1, 0);
		viewer->setSize(880, 550);

//		unsigned char red [13] = {   0,   0, 255, 255,   0, 130,   0,   0, 130, 130,   0, 130, 255};
//		unsigned char grn [13] = { 255,   0, 255,   0, 255,   0, 130,   0, 130,   0, 130, 130, 255};
//		unsigned char blu [13] = {   0, 255,   0, 255, 255,   0,   0, 130,   0, 130, 130, 130, 255};
		std::vector<unsigned char> red = 
			{ 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
			240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240 };
		std::vector<unsigned char> grn = 
			{ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
			255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
			255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255 };
		std::vector<unsigned char> blu = 
			{ 0, 20, 40, 60, 80, 100, 120, 140, 160, 180, 200, 220, 240,
			255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
			255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
			240, 220, 200, 180, 160, 140, 120, 100, 80, 60, 40, 20, 0,
			0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
		int N=red.size(), count=0;

		for(int i=0;i<timestamps_traj.size();i++)
		{
			if(stop) break;

			cur_time=timestamps_traj[i];

			if(debug) if(timestamps_traj[i]-debug_time<-1e-4) continue;
			if(!debug) 
			{
				if(start_time!=0)
				{
					if(timestamps_traj[i]<start_time) continue;
				}
				if(end_time!=0)
				{
					if(timestamps_traj[i]>end_time) break;
				}
			}

			if(last_time!=-1)
			{
				if(cur_time-last_time<delta_time) continue;
			}

			std::cout<<std::endl;
			std::cout<<"scan.timestamp = "<<std::fixed<<timestamps_traj[i]<<std::endl;

//			gettimeofday(&start,NULL);
//			usleep((cur_time-last_time)*1e6);
//			gettimeofday(&end,NULL);
//			timeused=(1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec)/1000;
//			cout<<"last for "<<timeused<<" ms\t"<<endl;

			scan_cur=new Scan(timestamps_traj[i]);
			scan_cur->loadScan(timestamps_traj[i],files_depth.find(timestamps_traj[i])->second,files_rgb.find(timestamps_traj[i])->second);
			scan_cur->Tcw()=Tcw_truth.find(timestamps_traj[i])->second;
			scan_cur->Tcg()=Tcg_traj.find(timestamps_traj[i])->second;

//			img_depth=cv::imread(files_depth.find(timestamps_traj[i])->second,cv::IMREAD_UNCHANGED);
//			img_rgb=cv::imread(files_rgb.find(timestamps_traj[i])->second,cv::IMREAD_UNCHANGED);
//			cv::imshow("rgb",img_rgb);
//			cv::waitKey(1);
//			cv::imshow("depth",img_depth);
//			cv::waitKey(1);

//			pcl::transformPointCloud(*scan_cur->points(),*cloud_scan,scan_cur->Tcg().inv().getMatrix4f());
//			vg.setInputCloud(cloud_scan);
//			vg.setLeafSize(0.01f,0.01f,0.01f);
//			vg.filter(*cloud_filtered);

			plane_extraction->extractPlanes(scan_cur,vis);
			plane_fitting->fitPlanes(scan_cur);
			edge_extraction->extract(scan_cur);

//			viewer->removeAllPointClouds();
			scan_cur->visScanFeatures(viewer);
//			viewer->spinOnce();
			viewer->spinOnce(1, true);

			if(scan_ref!=0)
			{
				scan_cur->ref()=scan_ref;

				it_gfm->match(scan_cur,vis);

				FeatureAssociation *fa=scan_cur->association();
				for(int j=0;j<fa->size();j++)
				{
					if(fa->getFeature(j)->Type()!=PLANE) continue;

					Plane* pr=fa->getFeatureRef(j)->plane();
					Plane* pc=fa->getFeature(j)->plane();

					if(!pr->in_map) 
					{
						int idx=count%N;
						pr->colorPlane(red[idx],grn[idx],blu[idx]);
						cout<<fa->getFeatureRef(j)<<": "<<(int)red[idx]<<", "<<(int)grn[idx]<<", "<<(int)blu[idx]<<endl;
						count+=10;
					}
					pc->colorPlane(pr->red,pr->grn,pr->blu);
					cout<<fa->getFeature(j)<<": "<<(int)pc->red<<", "<<(int)pc->grn<<", "<<(int)pc->blu<<endl;
				}
				scan_cur->Tcg().inv().vis(vis,scan_cur->time());
				scan_cur->visScanFeatures(vis,ESTIMATE);
	//			if (!vis->updatePointCloud(cloud_filtered,std::to_string(scan_cur->time())))
	//				vis->addPointCloud(cloud_filtered,std::to_string(scan_cur->time()));
				vis->spinOnce();

				delete scan_ref;
			}
			else 
			{
				for(iterFeature it=scan_cur->beginFeature();it!=scan_cur->endFeature();it++)
				{
					Plane* pl=it->second->plane();
					int idx=count%N;
					pl->colorPlane(red[idx],grn[idx],blu[idx]);
					count+=10;
				}
				scan_cur->Tcg().vis(vis,scan_cur->time());
				scan_cur->visScanFeatures(vis,ESTIMATE);
//				vis->spin();
			}
			scan_ref=scan_cur;

			if(debug || last_time==-1) vis->spin();

			last_time=cur_time;
//			delete scan_cur;
		}

		vis->spin();

		vis->removeAllCoordinateSystems();
		vis->removeAllPointClouds();

//		compareCloud(vis);

//		Eigen::Affine3f coords(Transform::Identity().getMatrix4f());
//		vis->addCoordinateSystem(1.0,coords,"CoordsSys");

//		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_gt(new pcl::PointCloud<pcl::PointXYZ>);
//		pcl::io::loadPCDFile<pcl::PointXYZ>("living-room.pcd",*cloud_gt);
//		for(int i=0;i<cloud_gt->size();i++)
//		{
//			cloud_gt->at(i).x=-cloud_gt->at(i).x;
//		}
//		vis->addPointCloud(cloud_gt,"lr");
//		vis->spin();

//		vg.setInputCloud(cloud);
//		vg.setLeafSize(0.01f,0.01f,0.01f);
//		vg.filter(*cloud);
//		vis->addPointCloud(cloud,"scan");
//		vis->spin(); 

//		pcl::PCDWriter writer;
//		writer.write<pcl::PointXYZRGBA>("pointcloud.pcd",*cloud,false);

		if(debug) fp.close();
		fp_time.close();
		return 0;
	}

	void PlaneEdgeSLAM::loadImages(const std::string &folder)
	{
		std::string file_association=folder+"/association.txt";

		std::ifstream fAssociation;
		fAssociation.open(file_association);
		while(!fAssociation.eof())
		{
			std::string s; getline(fAssociation,s);
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				double t,time; std::string sRGB, sD;
				ss>>time>>sD;
				ss>>t>>sRGB;
				double tx,ty,tz,qx,qy,qz,qw;
				ss>>t>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				ulysses::Transform Twc(tx,ty,tz,qx,qy,qz,qw);

				timestamps.push_back(time);
				Tcw_truth.insert(std::pair<double,Transform>(time,Twc.inv()));
				files_depth.insert(std::pair<double,std::string>(time,folder+"/"+sD));
				files_rgb.insert(std::pair<double,std::string>(time,folder+"/"+sRGB));
			}
		}
		fAssociation.close();
	}

	void PlaneEdgeSLAM::loadTraj(const std::string &folder)
	{
		std::string file=folder+"/traj.txt";
//		cout<<file<<endl;

		std::ifstream fp_traj;
		fp_traj.open(file);
		while(!fp_traj.eof())
		{
			std::string s; getline(fp_traj,s);
//			cout<<s<<endl;
			if(!s.empty())
			{
				std::stringstream ss; ss<<s;
				double t,time; std::string sRGB, sD;
				ss>>time;
				timestamps_traj.push_back(time);

				double tx,ty,tz,qx,qy,qz,qw;
				ss>>tx>>ty>>tz>>qx>>qy>>qz>>qw;
				ulysses::Transform Twc(tx,ty,tz,qx,qy,qz,qw);
				Tcg_traj.insert(std::pair<double,Transform>(time,Twc.inv()));
			}
		}
		fp_traj.close();
	}

}

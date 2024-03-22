/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-05-26 15:42
#
# Filename:		edge_point_extraction.h
#
# Description: 
#
===============================================*/
#ifndef _EDGE_H_
#define _EDGE_H_

#include "types/types.h"
#include "types/map.h"

namespace ulysses
{
	extern CameraIntrinsic camera_intrinsic;

	class EdgeExtraction : public pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>
	{
	public:
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_NAN_BOUNDARY;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDING;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_OCCLUDED;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_HIGH_CURVATURE;
		using OrganizedEdgeBase<pcl::PointXYZRGBA, pcl::Label>::EDGELABEL_RGB_CANNY;	

		EdgeExtraction(const std::string &settingFile)
		{ 
			remove("edge_point_extraction.txt");
			cv::FileStorage settings(settingFile.c_str(),cv::FileStorage::READ);
			settings["debug"]>>debug; 
			bool useCanny; settings["EdgeExtraction.useCanny"]>>useCanny; 
			bool useHighCurv; settings["EdgeExtraction.useHighCurv"]>>useHighCurv; 
			bool useOccluding; settings["EdgeExtraction.useOccluding"]>>useOccluding; 
			settings["EdgeExtraction.win_size"]>>win_size; 
			settings["EdgeExtraction.ratio"]>>ratio; 
			settings["EdgeExtraction.depth_thres"]>>depth_thres; 
//			rho				   = (double)settings["LineExtraction.HoughLinesP.rho"]; 
//			theta			   = (double)settings["LineExtraction.HoughLinesP.theta"]*M_PI/180.0; 
//			threshold		   = (int)settings["LineExtraction.HoughLinesP.threshold"]; 
//			minLineLength	   = (double)settings["LineExtraction.HoughLinesP.minLineLength"]; 
//			maxLineGap		   = (double)settings["LineExtraction.HoughLinesP.maxLineGap"];
//			setDepthDisconThreshold ((double)settings["EdgeExtraction.PCLOrganizedEdge.DepthDisconThreshold"]);
//			setMaxSearchNeighbors ((int)settings["EdgeExtraction.PCLOrganizedEdge.MaxSearchNeighbors"]);
//			setHCCannyLowThreshold ((double)settings["EdgeExtraction.PCLOrganizedEdge.HCCannyLowThreshold"]);
//			setHCCannyHighThreshold((double)settings["EdgeExtraction.PCLOrganizedEdge.HCCannyHighThreshold"]);
			settings.release();
			int edge_types=0;
			if(useOccluding) edge_types=edge_types|EDGELABEL_OCCLUDING;
			if(useHighCurv) edge_types=edge_types|EDGELABEL_HIGH_CURVATURE;
			if(useCanny) edge_types=edge_types|EDGELABEL_RGB_CANNY;
//			if(useCanny) setEdgeType (EDGELABEL_OCCLUDING | EDGELABEL_HIGH_CURVATURE | EDGELABEL_RGB_CANNY); 
//			else if(useHighCurv) setEdgeType (EDGELABEL_OCCLUDING | EDGELABEL_HIGH_CURVATURE); 
//			else setEdgeType (EDGELABEL_OCCLUDING); 
			if(edge_types==0) edge_types=EDGELABEL_OCCLUDED;
			setEdgeType(edge_types);
		}

		~EdgeExtraction() {}

		void setDebug(bool d) {debug=d;}

		void extract(Scan *scan);

		void vis(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis);

	private:
		
		std::ofstream fp;
		bool debug;

		int win_size;
		double ratio, depth_thres;

		void addEdgePoints(Scan *scan, const pcl::PointIndices &edge_indices);
	};

}

#endif

/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-05-26 15:41
#
# Filename:		edge_point_extraction.cpp
#
# Description: 
#
===============================================*/

#include "features/edge_extraction.h"

namespace ulysses
{
	void EdgeExtraction::extract(Scan *scan)
	{
		if(debug) fp.open("edge_point_extraction.txt",std::ios::app);
		if(debug) fp<<std::endl<<"******************************************************************"<<std::endl;

		// edge
		pcl::PointCloud<pcl::Label>::Ptr labels_edge=pcl::PointCloud<pcl::Label>::Ptr (new pcl::PointCloud<pcl::Label>);
		std::vector<pcl::PointIndices> edge_indices;

		// for edge detection;
		// change the invalid depth in scan->point_cloud from zero to infinite;
		pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGBA>);
		*cloud_tmp=*scan->points();
		for(size_t i=0;i<cloud_tmp->height;i++)
		{
			for(size_t j=0;j<cloud_tmp->width;j++)
			{
				double dep=cloud_tmp->points[cloud_tmp->width*i+j].z;
				if(std::abs(dep)<1e-4)
				{
					cloud_tmp->points[cloud_tmp->width*i+j].z=std::numeric_limits<double>::max();
				}
			}
		}

		// edge detection;
		if (getEdgeType () & EDGELABEL_HIGH_CURVATURE)
		{
			pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputNormals(scan->normals());
		}
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::setInputCloud(cloud_tmp);
		pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::compute(*labels_edge, edge_indices);

		if(debug)
		{
			fp<<"getDepthDisconThreshold - "<<getDepthDisconThreshold()<<std::endl;
			fp<<"getMaxSearchNeighbors   - "<<getMaxSearchNeighbors() <<std::endl;
			fp<<"getHCCannyLowThreshold  - "<<getHCCannyLowThreshold() <<std::endl;
			fp<<"getHCCannyHighThreshold - "<<getHCCannyHighThreshold()<<std::endl<<std::endl;

			fp<<"organized edge detection "<<std::endl;
			fp<<"\tEDGELABEL_OCCLUDING - "<<edge_indices[1].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_HIGH_CURVATURE - "<<edge_indices[3].indices.size()<<std::endl;
			fp<<"\tEDGELABEL_RGB_CANNY - "<<edge_indices[4].indices.size()<<std::endl;
		}

		addEdgePoints(scan,edge_indices[1]); // occluding edge;
		addEdgePoints(scan,edge_indices[3]); // high curvature edge;
		addEdgePoints(scan,edge_indices[4]); // rgb canny edge;

//		if(scan->sizeEdgePoint()<1000)
//		{
//			setEdgeType(EDGELABEL_RGB_CANNY); 
//			pcl::OrganizedEdgeFromRGBNormals<pcl::PointXYZRGBA, pcl::Normal, pcl::Label>::compute(*labels_edge, edge_indices);
//		}

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		cloud->points.resize(scan->sizeEdgePoint());
		for(int i=0;i<scan->sizeEdgePoint();i++)
		{
			cloud->points[i].x=scan->edgePoint(i)->xyz(0);
			cloud->points[i].y=scan->edgePoint(i)->xyz(1);
			cloud->points[i].z=scan->edgePoint(i)->xyz(2);
		}

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud);
		pcl::PointXYZ searchPoint;
		std::vector<int> pointIdx;
		std::vector<float> SquaredDistance;
		float radius=0.05;

		for(int i=0;i<scan->sizeEdgePoint();i++)
		{
			searchPoint.x=scan->edgePoint(i)->xyz(0);
			searchPoint.y=scan->edgePoint(i)->xyz(1);
			searchPoint.z=scan->edgePoint(i)->xyz(2);
			if(kdtree.radiusSearch(searchPoint,radius,pointIdx,SquaredDistance)>0)
			{
				if(pointIdx.size()>7)
				{
					for(int j=0;j<pointIdx.size();j++)
					{
						scan->edgePoint(i)->neighbors.push_back(scan->edgePoint(pointIdx[j]));
					}
					scan->edgePoint(i)->isEdge=true;
					scan->edgePoint(i)->computeInfo();
				}
			}
		}

		if(debug) fp.close();
	}

	void EdgeExtraction::addEdgePoints(Scan *scan, const pcl::PointIndices &edge_indices)
	{
		if(edge_indices.indices.size()==0) return;

		int height=camera_intrinsic.height, width=camera_intrinsic.width;
		cv::Mat img=cv::Mat::zeros(height,width,CV_8UC1);

		// occluding edge;
		for(int i=0;i<edge_indices.indices.size();i++)
		{
			int idx=edge_indices.indices[i];

			int x=scan->pixels()->at(idx).x; // width
			int y=scan->pixels()->at(idx).y; // height
//			img.at<cv::Vec3b>(y,x)[0]=255;
//			img.at<cv::Vec3b>(y,x)[1]=0;
//			img.at<cv::Vec3b>(y,x)[2]=0;
			img.at<unsigned char>(y,x)=255;

//			scan->addEdgePoint(idx);
		}
		if(debug)
		{
			cv::imshow("edge",img);
			cv::waitKey(0);
		}

		cv::Mat filtered=cv::Mat::zeros(height,width,CV_8UC1);
		for(int i=0;i<edge_indices.indices.size();i++)
		{
			int idx=edge_indices.indices[i];
			int x=scan->pixels()->at(idx).x; // width
			int y=scan->pixels()->at(idx).y; // height

			if(x-win_size<=0 || y-win_size<0 || x+win_size>=width || y+win_size>=height) continue;
			if(scan->points()->at(idx).z>depth_thres) continue;

			Eigen::Matrix2d scatter=Eigen::Matrix2d::Zero();
			Eigen::Vector2d centroid(y,x);
			int num=0;
			for(int j=x-win_size;j<x+win_size;j++)
			{
				for(int k=y-win_size;k<y+win_size;k++)
				{
					if(img.at<unsigned char>(k,j)==255)
					{
						Eigen::Vector2d tmp(k,j);
						tmp-=centroid;
						scatter+=tmp*tmp.transpose();
						num++;
					}
				}
			}
			if(num==0) continue;
			scatter/=num;
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(scatter);
			Eigen::Vector2d ev=es.eigenvalues();
			ev=ev.cwiseAbs();
//			fp<<ev.transpose()<<endl;
			if((ev(0)>ev(1) && ev(1)/ev(0)<ratio) || (ev(1)>ev(0) && ev(0)/ev(1)<ratio))
			{
				filtered.at<unsigned char>(y,x)=255;
				scan->addEdgePoint(idx);
			}
		}
		if(debug)
		{
			cv::imshow("edge",filtered);
			cv::waitKey(0);
		}
	}

}


/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-05-19 13:10
#
# Filename:		plane_fitting.cpp
#
# Description: 
#
===============================================*/

#include <features/plane_fitting.h>


namespace ulysses
{
	void PlaneFitting::fitPlanes(Scan *scan)
	{
		for(iterFeature it=scan->beginFeature();it!=scan->endFeature();it++)
		{
			if(it->second->Type()==PLANE)
			{
				fitPlaneModel(it->second->plane());
			}
		}
	}

	void PlaneFitting::fitPlaneModel(Plane *plane)
	{
		fp.open("plane_fitting.txt",std::ios::app);
		if(debug)
			fp<<std::endl<<"************************fitPlane*********************************"<<std::endl;

		computeCentroidScatter(plane);
		EigenSolve(plane->scatter_point,plane->n);
		if(plane->n.transpose()*plane->centroid_point>0) plane->n=-plane->n;
		plane->d=-plane->n.transpose()*plane->centroid_point;
		if(debug) fp<<"fitted LS plane model: ("<<plane->n.transpose()<<","<<plane->d<<")"<<std::endl;
//		std::cout<<"fitted LS plane model: ("<<plane->n.transpose()<<","<<plane->d<<")"<<std::endl;

		computePointWeight(plane,plane->n);

		bool weighted = (fitting_method!="LS");

		if(weighted)
		{
			computeCentroidScatter(plane,weighted);
			EigenSolve(plane->scatter_point,plane->n);
			if(plane->n.transpose()*plane->centroid_point>0) plane->n=-plane->n;
			plane->d=-plane->n.transpose()*plane->centroid_point;
			if(debug) fp<<"fitted probablistic plane model: ("<<plane->n.transpose()<<","<<plane->d<<")"<<std::endl;
//			std::cout<<"fitted probablistic plane model: ("<<plane->n.transpose()<<","<<plane->d<<")"<<std::endl;
		}

		computePlaneInfo(plane,weighted);

		fp.close();
	}

	void PlaneFitting::removeOutliers(Plane *plane)
	{
		Eigen::Vector3d n=plane->n;
		double d=plane->d;
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			Eigen::Vector3d p;
			p(0)=plane->ptr_points->at(idx).x;
			p(1)=plane->ptr_points->at(idx).y;
			p(2)=plane->ptr_points->at(idx).z;
			if(fabs(n.dot(p)+d)>0.02) plane->indices[i]=0;
		}
	}


	void PlaneFitting::EigenSolve(const Eigen::Matrix3d &scatter, Eigen::Vector3d &n)
	{
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(scatter);
		Eigen::Vector3d eig_vals=es.eigenvalues();
		Eigen::Matrix3d eig_vecs=es.eigenvectors();
		eig_vals=eig_vals.cwiseAbs();
		double min=DBL_MAX;
		int idx=-1;
		for(int i=0;i<3;i++)
		{
			if(eig_vals(i)<min)
			{
				min=eig_vals(i);
				idx=i;
			}
		}
		n=eig_vecs.block<3,1>(0,idx);
	}

	void PlaneFitting::computePointWeight(Plane *plane, Eigen::Vector3d n)
	{
		plane->weights.resize(plane->indices.size());
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			if(idx==0) continue;
			Eigen::Vector3d point;
			point(0)=plane->ptr_points->at(idx).x;
			point(1)=plane->ptr_points->at(idx).y;
			point(2)=plane->ptr_points->at(idx).z;
//			std::cout<<"point = "<<point.transpose()<<std::endl;
			if(fitting_method=="LS")
			{
				plane->weights[i]=1;
			}
			else if(fitting_method=="PR")
			{
//				Eigen::Matrix3d K=camera_intrinsic.getMatrix();
//				Eigen::Matrix3d K_inv=K.inverse();
//				Eigen::Vector3d u(plane->ptr_pixels->at(idx).x,plane->ptr_pixels->at(idx).y,1);
//				Eigen::Vector3d Ku=K_inv*u;
//				double sigma_z=camera_intrinsic.m_fp*camera_intrinsic.sigma_disparity*point(2)*point(2);
//				Eigen::Matrix3d cov=Ku*sigma_z*Ku.transpose();
//				plane->weights[i]=1.0/(n.dot(cov*n));
//			}
//			else if(fitting_method=="PR_pxl")
//			{
				Eigen::Matrix3d K=camera_intrinsic.getMatrix();
				Eigen::Matrix3d K_inv=K.inverse();
				Eigen::Vector3d u(plane->ptr_pixels->at(idx).x,plane->ptr_pixels->at(idx).y,1);
				Eigen::Vector3d Ku=K_inv*u;
				double sigma_z=camera_intrinsic.m_fp*camera_intrinsic.sigma_disparity*point(2)*point(2);
				Eigen::Matrix3d C_u=Eigen::Matrix3d::Zero();
				C_u(0,0)=camera_intrinsic.sigma_u*camera_intrinsic.sigma_u;
				C_u(1,1)=camera_intrinsic.sigma_v*camera_intrinsic.sigma_v;
				Eigen::Matrix3d cov=Ku*sigma_z*Ku.transpose()+point(2)*point(2)*K_inv*C_u*K_inv.transpose();
				plane->weights[i]=1/(n.dot(cov*n));
			}

		}
	}

	void PlaneFitting::computeCentroidScatter(Plane *plane, bool weighted)
	{
		Eigen::Vector3d tmp;
		plane->centroid_point.setZero();
		plane->centroid_color.setZero();
		plane->scatter_point.setZero();
		plane->scatter_color.setZero();
		double weight=1.0, sum=0.0;
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			if(idx==0) continue;
			if(weighted) weight=plane->weights[i];
			plane->centroid_point(0)+=weight*plane->ptr_points->at(idx).x;
			plane->centroid_point(1)+=weight*plane->ptr_points->at(idx).y;
			plane->centroid_point(2)+=weight*plane->ptr_points->at(idx).z;
			plane->centroid_color(0)+=weight*double(plane->ptr_points->at(idx).r)/255.0;
			plane->centroid_color(1)+=weight*double(plane->ptr_points->at(idx).g)/255.0;
			plane->centroid_color(2)+=weight*double(plane->ptr_points->at(idx).b)/255.0;
			sum+=weight;
		}
		plane->centroid_point/=sum;
		plane->centroid_color/=sum;
		sum=0.0;
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			if(idx==0) continue;
			if(weighted) weight=plane->weights[i];
			Eigen::Vector3d tmp;
			tmp(0)=plane->ptr_points->at(idx).x-plane->centroid_point(0);
			tmp(1)=plane->ptr_points->at(idx).y-plane->centroid_point(1);
			tmp(2)=plane->ptr_points->at(idx).z-plane->centroid_point(2);
			plane->scatter_point+=weight*tmp*tmp.transpose();
			tmp(0)=double(plane->ptr_points->at(idx).r)/255.0-plane->centroid_color(0);
			tmp(1)=double(plane->ptr_points->at(idx).g)/255.0-plane->centroid_color(1);
			tmp(2)=double(plane->ptr_points->at(idx).b)/255.0-plane->centroid_color(2);
			plane->scatter_color+=weight*tmp*tmp.transpose();
			sum+=weight;
		}
		plane->scatter_point/=sum;
		plane->scatter_color/=sum;

		if(debug)
		{
			fp<<"plane->centroid_point: "<<plane->centroid_point.transpose()<<std::endl;
			fp<<"plane->scatter_point: "<<std::endl<<plane->scatter_point<<std::endl;
			fp<<"plane->centroid_color: "<<plane->centroid_color.transpose()<<std::endl;
			fp<<"plane->scatter_color: "<<std::endl<<plane->scatter_color<<std::endl;
		}
	}

	void PlaneFitting::computePlaneInfo(Plane *plane, bool weighted)
	{
		plane->info.setZero();
		Eigen::Matrix4d tmp;
		double weight=1.0;
		for(int i=0;i<plane->indices.size();i++)
		{
			int idx=plane->indices[i];
			if(idx==0) continue;
			if(weighted) weight=plane->weights[i];
			Eigen::Vector3d pt;
			pt(0)=plane->ptr_points->at(idx).x;
			pt(1)=plane->ptr_points->at(idx).y;
			pt(2)=plane->ptr_points->at(idx).z;
			tmp.block<3,3>(0,0)=pt*pt.transpose();
			tmp.block<3,1>(0,3)=pt;
			tmp.block<1,3>(3,0)=pt.transpose();
			tmp(3,3)=1;
			tmp=tmp*weight;
			plane->info+=tmp;
		}
		//plane->info/=plane->indices.size();
		if(debug) fp<<"plane->info: "<<std::endl<<plane->info<<std::endl;

		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> es;
		es.compute(plane->info);
		Eigen::Vector4d Lambda=es.eigenvalues(); Lambda=Lambda.cwiseAbs();
		Eigen::Matrix4d U=es.eigenvectors();
		Eigen::Matrix4d sqrt_Lambda=Eigen::Matrix4d::Zero();
		Eigen::Matrix4d inv_Lambda=Eigen::Matrix4d::Zero();
		for(int i=0;i<4;i++)
		{
			sqrt_Lambda(i,i)=sqrt(Lambda(i));
			if(Lambda(i)>0.01)
			{
				inv_Lambda(i,i)=1.0/Lambda(i);
			}
		}
		plane->sqrt_info=U*sqrt_Lambda*U.transpose();
		plane->cov=U*inv_Lambda*U.transpose();
		if(debug) fp<<"plane->sqrt_info: "<<std::endl<<plane->sqrt_info<<std::endl;
		if(debug) fp<<"plane->cov: "<<std::endl<<plane->cov<<std::endl;
	}
}

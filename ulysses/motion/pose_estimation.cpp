/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-05-26 10:50
#
# Filename:		pose_estimation.cpp
#
# Description: 
#
===============================================*/

#include "pose_estimation.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace ulysses
{

	void Edge_Plane::computeError()
	{
		const g2o::VertexSE3Expmap* v0 =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
		const g2o::VertexSE3Expmap* v1 =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
		// Tcr=Tcg*Trg.inv();
		g2o::SE3Quat Tcr=v0->estimate().inverse()*v1->estimate();
		Eigen::Vector4d plane_ref_trans=_measurement.transform_pln_ref(Tcr);
		_error=_measurement.pln_cur-plane_ref_trans;
	}

//	void Edge_Plane::linearizeOplus()
//	{
//		g2o::VertexSE3Expmap* vp0 = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
//		g2o::VertexSE3Expmap* vp1 = static_cast<g2o::VertexSE3Expmap*>(_vertices[1]);
//
//		if (!vp0->fixed())
//		{
//			_jacobianOplusXi.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
//			_jacobianOplusXi.block<3,3>(0,3) = -g2o::skew(_measurement.pln_cur.block<3,1>(0,0));
//			_jacobianOplusXi.block<1,3>(3,0) = -_measurement.pln_cur.block<3,1>(0,0).transpose();
//			_jacobianOplusXi.block<1,3>(3,3) = Eigen::Vector3d::Zero().transpose();
//		}
//
//		if (!vp1->fixed())
//		{
//			_jacobianOplusXj.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
//			_jacobianOplusXj.block<3,3>(0,3) = g2o::skew(_measurement.pln_ref.block<3,1>(0,0));
//			_jacobianOplusXj.block<1,3>(3,0) = _measurement.pln_ref.block<3,1>(0,0).transpose();
//			_jacobianOplusXj.block<1,3>(3,3) = Eigen::Vector3d::Zero().transpose();
//		}
//	}

	void Edge_EdgePoint::computeError()
	{
		const g2o::VertexSE3Expmap* v0 =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
		const g2o::VertexSE3Expmap* v1 =static_cast<const g2o::VertexSE3Expmap*> ( _vertices[1] );
		// Tcr=Tcg*Trg.inv();
		g2o::SE3Quat Tcr=v0->estimate().inverse()*v1->estimate();
		Eigen::Vector3d point_ref_trans=_measurement.transform_pt_ref(Tcr);
		_error=_measurement.pt_cur-point_ref_trans;
	}

//	Transform PoseEstimation::alignPlanes(std::vector<PlanePair> matched_planes)
//	{
//		std::ofstream fp;
//		fp.open("alignPlanes.txt",std::ios::app);
//		if(debug)
//			fp<<std::endl<<"*********************************************************"<<std::endl;
//		Transform Tcr_align_planes;
//		_case=constraint_case(matched_planes);
//		if(debug)
//		{
//			fp<<"constrain case - "<<_case<<std::endl;
//			fp<<"svd of H - "<<H_singularValues.transpose()<<std::endl;
//			fp<<H_svd_U<<std::endl;
//			fp<<H_svd_V<<std::endl;
//		}
//		Eigen::Matrix3d tmp_mat3d,tmp_inverse;
//		bool invertible;
//		Tcr_align_planes.R=H_svd_V*H_svd_U.transpose(); // R_cr
//		if(debug)
//			fp<<"Rotation - "<<std::endl<<Tcr_align_planes.R<<std::endl;
//		Eigen::MatrixXd A=Eigen::MatrixXd::Zero(MAX_PLNS,3);
//		Eigen::VectorXd d=Eigen::VectorXd::Zero(MAX_PLNS);
//		if(_case==DoF_6)
//		{
//			//Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
//			for(int i=0;i<matched_planes.size();i++)
//			{
//				A.block<1,3>(i,0)=matched_planes[i].cur->normal.transpose();
//				d(i)=matched_planes[i].ref->d-matched_planes[i].cur->d;
//			}
//			if(debug)
//			{
//				fp<<"A - "<<std::endl<<A<<std::endl;
//				fp<<"d - "<<d.transpose()<<std::endl;
//			}
//		}
//		else if(_case==DoF_5)
//		{
//			//Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
//			if(abs(Tcr_align_planes.R.determinant()+1.0f)<1.0e-4)
//			{
//				H_svd_U.block<3,1>(0,2)=-H_svd_U.block<3,1>(0,2);
//				Tcr_align_planes.R=H_svd_U*H_svd_V.transpose();
//				if(debug)
//				{
//					fp<<"U':"<<std::endl<<H_svd_U<<std::endl;
//					fp<<"det(R'):"<<Tcr_align_planes.R.determinant()<<std::endl;
//				}
//			}
//			else
//			{
//				if(debug)
//				{
//					fp<<"U:"<<std::endl<<H_svd_U<<std::endl;
//					fp<<"det(R):"<<Tcr_align_planes.R.determinant()<<std::endl;
//				}
//			}
//			for(int i=0;i<matched_planes.size();i++)
//			{
//				A.block<1,3>(i,0)=matched_planes[i].cur->normal.transpose();
//				d(i)=matched_planes[i].ref->d-matched_planes[i].cur->d;
//			}
//			A.block<1,3>(matched_planes.size(),0)=H_svd_V.block<3,1>(0,2).transpose();
//		}
//		else if(_case==DoF_3)
//		{
//			Eigen::Matrix3d H1, H_svd_U1, H_svd_V1;
//			H1=H+H_svd_U.block<3,1>(0,2)*H_svd_V.block<3,1>(0,2).transpose();
//			Eigen::JacobiSVD<Eigen::Matrix3d> svd(H1, Eigen::ComputeFullU | Eigen::ComputeFullV);
//			H_svd_U1=svd.matrixU();
//			H_svd_V1=svd.matrixV();
//			Tcr_align_planes.R=H_svd_U1*H_svd_V1.transpose();
//			if(abs(Tcr_align_planes.R.determinant()+1.0f)<1.0e-4)
//			{
//				H_svd_U1.block<3,1>(0,2)=-H_svd_U1.block<3,1>(0,2);
//				Tcr_align_planes.R=H_svd_U1*H_svd_V1.transpose();
//				if(debug)
//				{
//					fp<<"U1':"<<std::endl<<H_svd_U1<<std::endl;
//					fp<<"det(R'):"<<Tcr_align_planes.R.determinant()<<std::endl;
//				}
//			}
//			else
//			{
//				if(debug)
//				{
//					fp<<"U1:"<<std::endl<<H_svd_U1<<std::endl;
//					fp<<"det(R):"<<Tcr_align_planes.R.determinant()<<std::endl;
//				}
//			}
//			for(int i=0;i<matched_planes.size();i++)
//			{
//				A.block<1,3>(i,0)=matched_planes[i].cur->normal.transpose();
//				d(i)=matched_planes[i].ref->d-matched_planes[i].cur->d;
//			}
//			A.block<1,3>(matched_planes.size(),0)=H_svd_V.block<3,1>(0,1).transpose();
//			A.block<1,3>(matched_planes.size()+1,0)=H_svd_V.block<3,1>(0,2).transpose();
//		}
//		tmp_mat3d=A.transpose()*A;
//		tmp_mat3d.computeInverseWithCheck(tmp_inverse,invertible);
//		if(invertible)
//		{
//			Tcr_align_planes.t=tmp_inverse*A.transpose()*d;
//			if(debug)
//			{
//				fp<<"inv(ATA) - "<<std::endl<<tmp_inverse<<std::endl;
//				fp<<"translation - "<<Tcr_align_planes.t.transpose()<<std::endl;
//			}
//		}
//		else
//		{
//			std::cerr<<"matrix A uninvertible!"<<std::endl;
//			return Transform();
//		}
//		fp.close();
//		return Tcr_align_planes;
//	}

//
//	PoseEstimation::ConstraintCase PoseEstimation::constraint_case(std::vector<PlanePair> matched_planes)
//	{
//		ofstream fp;
//		fp.open("constraint_case.txt",std::ios::app);
//		H.setZero();
//		for(int i=0;i<matched_planes.size();i++)
//		{
//			H=H+matched_planes[i].ref->normal*matched_planes[i].cur->normal.transpose();
//		}
//		if(debug)
//			fp<<"H - "<<std::endl<<H<<std::endl;
//
//		Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
//		H_svd_U=svd.matrixU();
//		H_svd_V=svd.matrixV();
//		H_singularValues=svd.singularValues();
//		if(debug)
//		{
//			fp<<"singular - "<<H_singularValues.transpose()<<std::endl;
//			fp<<"H_svd_U - "<<std::endl<<H_svd_U<<std::endl;
//			fp<<"H_svd_V - "<<std::endl<<H_svd_V<<std::endl;
//		}
//
//		if(H_singularValues(0)>100*H_singularValues(1))
//			return DoF_3;
//		else if(H_singularValues(1)>100*H_singularValues(2))
//			return DoF_5;
//		else
//			return DoF_6;
//	}

	using namespace std;
	void PoseEstimation::estimate(Scan *scan_cur, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
		char id[20];
		fp.open("pose_estimation.txt",std::ios::app);
		if(debug) fp<<std::endl<<"******************************************************************"<<std::endl;
		int iterations = 0;
		bool converged = false;
		double chi2_pre=DBL_MAX, chi2=DBL_MAX;
		Transform Tcr=Transform::Identity();
		
//		confirmPlaneCorrespondence(scan_cur,Tcr);

		while(iterations<max_iter_icp && !converged)
		{
			if(debug)
			{
				fp<<std::endl<<"========================================"<<std::endl;
				fp<<"icp iteration - "<<iterations<<std::endl;
				fp<<"Tcr = "<<Tcr<<std::endl;
			}
//			cout<<"icp iteration - "<<iterations<<std::endl;
//			cout<<"Tcr = "<<Tcr<<endl;

			buildCorrespondence(scan_cur,Tcr);
//			cout<<"edge point associated"<<endl;
			//if(scan_cur->plane_matches.size()==0 || scan_cur->point_matches.size()<10)
			//	return -1;
//			confirmPlaneCorrespondence(scan_cur,Tcr);

//			compute_Psi_pi(scan_cur,Tcr);
//			compute_EdgePoint_weight(scan_cur,Tcr);

			chi2_pre=chi2;
			chi2=optimize(scan_cur,Tcr);
//			std::cout<<iterations<<" - "<<chi2<<std::endl;
//			cout<<"Tcr = "<<Tcr<<endl;
			
			iterations++;
			if(chi2>=chi2_pre)
				converged=true;
		}
		scan_cur->Tcr()=Tcr;
//		scan_cur->localize();

		fp.close();

//		double error=0;
//		for(size_t i=0;i<scan_cur->point_matches.size();i++)
//		{
//			Eigen::Vector3d vec3d=scan_cur->point_matches[i].cur->xyz;
//			vec3d=vec3d-scan_cur->point_matches[i].ref->xyz;
//			error+=vec3d.norm();
//		}
//		error/=scan_cur->point_matches.size();
//
//		return error;
	}



	double PoseEstimation::optimize(Scan *scan_cur, Transform &Tcr)
	{
		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,6>> DirectBlock;
		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverEigen< DirectBlock::PoseMatrixType > ();
//		std::unique_ptr<g2o::LinearSolverDense<DirectBlock::PoseMatrixType> > linearSolver 
//			= g2o::make_unique<g2o::LinearSolverDense<DirectBlock::PoseMatrixType> >();
		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
//		DirectBlock* solver_ptr = new DirectBlock ( std::unique_ptr<DirectBlock::LinearSolverType>(linearSolver) );
//		std::unique_ptr<DirectBlock> solver_ptr = g2o::make_unique<DirectBlock>(std::move(linearSolver));
		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
//		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<DirectBlock>(solver_ptr) ); // L-M
//		g2o::OptimizationAlgorithmLevenberg* solver 
//			= new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr) ); // L-M
		g2o::SparseOptimizer optimizer;
		optimizer.setAlgorithm ( solver );
		optimizer.setVerbose(true);
		cout<<"optimizer"<<endl;

		g2o::VertexSE3Expmap* pose0 = new g2o::VertexSE3Expmap();
		g2o::SE3Quat Tcr_g2o(Tcr.R,Tcr.t);
		pose0->setEstimate(Tcr_g2o.inverse());
		pose0->setId(0);
		pose0->setFixed(false);
		optimizer.addVertex(pose0);

		g2o::VertexSE3Expmap* pose1 = new g2o::VertexSE3Expmap();
		g2o::SE3Quat Trg(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());
		pose1->setEstimate(Trg.inverse());
		pose1->setId(1);
		pose1->setFixed(true);
		optimizer.addVertex(pose1);

		FeatureAssociation *fa=scan_cur->association();
		if(fa==0) usePlane=false;
		cout<<"usePlane: "<<usePlane<<endl;

		if(usePlane)
		for(size_t i=0;i<fa->size();i++)
		{
			MeasurementPlane measure_pln;
			measure_pln.pln_cur.block<3,1>(0,0)=fa->getFeature(i)->getDirection();
			measure_pln.pln_cur(3)=fa->getFeature(i)->getDistance().norm();
			measure_pln.pln_ref.block<3,1>(0,0)=fa->getFeatureRef(i)->getDirection();
			measure_pln.pln_ref(3)=fa->getFeatureRef(i)->getDistance().norm();

			Edge_Plane *edge=new Edge_Plane();
			edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(pose0));
			edge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(pose1));
			edge->setMeasurement(measure_pln);
			edge->setInformation(fa->getFeature(i)->plane()->info);
			edge->setId(i);
			optimizer.addEdge(edge);
		}

		if(useEdge)
		for(size_t i=0;i<edge_points_ref.size();i++)
		{
			if(edge_points_ref[i]==0) continue;

			MeasurementPoint measure_pt;
			measure_pt.pt_cur=scan_cur->edgePoint(i)->xyz;
			measure_pt.pt_ref=edge_points_ref[i]->xyz;

			Edge_EdgePoint *edge=new Edge_EdgePoint();
			edge->setVertex(0,dynamic_cast<g2o::OptimizableGraph::Vertex*>(pose0));
			edge->setVertex(1,dynamic_cast<g2o::OptimizableGraph::Vertex*>(pose1));
			edge->setMeasurement(measure_pt);
			edge->setInformation(scan_cur->edgePoint(i)->info);
//			fp<<scan_cur->edgePoint(i)->info<<endl<<endl;
//			edge->setInformation(scan_cur->point_matches[i].computeCov(Tcr).inverse()*Weight_p);
			edge->setId(fa->size()+i);
			optimizer.addEdge(edge);
		}
		cout<<"edges"<<endl;

		optimizer.initializeOptimization();
		cout<<"initialize"<<endl;
		int result=optimizer.optimize (max_iter_lm);

		Tcr_g2o = pose0->estimate().inverse();
		Tcr.R = Tcr_g2o.rotation().toRotationMatrix();
		Tcr.t = Tcr_g2o.translation();
		cout<<"Tcr: "<<Tcr<<endl;
//
//			if(debug)
//			{
//				int edge_idx=0;
//				fpd<<"edges - "<<optimizer.activeEdges().size()<<std::endl;
//				fpd<<"plane errors "<<std::endl;
//				for(size_t i=0;i<scan_cur->plane_matches.size();i++)
//				{
//					if(scan_cur->plane_matches[i].cur==0)
//						continue;
//					fpd<<"\t"<<i<<" - "<<optimizer.activeEdges()[edge_idx]->chi2()<<std::endl;
//					edge_idx++;
//				}
//				fpd<<"point errors "<<std::endl;
//				for(size_t i=0;i<scan_cur->point_matches.size();i++)
//				{
//					if(scan_cur->point_matches[i].weight<thres_weight)
//						continue;
//					fpd<<"\t"<<i<<" - "<<optimizer.activeEdges()[edge_idx]->chi2()<<std::endl;
//					edge_idx++;
//				}
//			}

		double chi2=optimizer.activeChi2();
		optimizer.clear(); // delete vertices and edges;
//		delete linearSolver;
//		delete solver_ptr;
//		delete solver;

		return chi2;
	}

	void PoseEstimation::confirmPlaneCorrespondence(Scan *scan_cur, Transform &Tcr) {}
//	{
//		typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;
//		DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
////		DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
//		DirectBlock* solver_ptr = new DirectBlock ( std::unique_ptr<DirectBlock::LinearSolverType>(linearSolver) );
//		//g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( solver_ptr ); // G-N
////		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr ); // L-M
//		g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::unique_ptr<DirectBlock>(solver_ptr) ); // L-M
//		g2o::SparseOptimizer optimizer;
//		optimizer.setAlgorithm ( solver );
//		optimizer.setVerbose(false);
//
//		g2o::VertexSE3Expmap* pose0 = new g2o::VertexSE3Expmap();
//		g2o::SE3Quat Tcg(Tcr.R,Tcr.t);
//		pose0->setEstimate (Tcg.inverse());
//		pose0->setId ( 0 );
//		pose0->setFixed(false);
//		optimizer.addVertex ( pose0 );
//
//		g2o::VertexSE3Expmap* pose1 = new g2o::VertexSE3Expmap();
//		g2o::SE3Quat Trg(Eigen::Matrix3d::Identity(),Eigen::Vector3d::Zero());
//		pose1->setEstimate (Trg.inverse());
//		pose1->setId ( 1 );
//		pose1->setFixed(true);
//		optimizer.addVertex ( pose1 );
//
//		FeatureAssociation *fa=scan_cur->association();
//		for(int i=0;i<fa->size();i++)
////		for(size_t i=0;i<scan_cur->plane_matches.size();i++)
//		{
//			if(fa->getFeature(i)->Type()!=PLANE) continue;
////			if(scan_cur->plane_matches[i].cur==0)
////				continue;
//			MeasurementPlane measure_pln;
//			measure_pln.pln_cur.block<3,1>(0,0)=fa->getFeature(i)->getDirection();
//			measure_pln.pln_cur(3)=fa->getFeature(i)->getDistance().norm();
//			measure_pln.pln_ref.block<3,1>(0,0)=fa->getFeatureRef(i)->getDirection();
//			measure_pln.pln_ref(3)=fa->getFeatureRef(i)->getDistance().norm();
//
//			Edge_Plane *edge=new Edge_Plane();
//			edge->setVertex(0,pose0);
//			edge->setVertex(1,pose1);
//			edge->setMeasurement(measure_pln);
////			edge->setInformation(scan_cur->plane_matches[i].cur->cov_inv);
////			edge->setInformation(scan_cur->plane_matches[i].computeCov(Tcr).inverse());
//			edge->setInformation(fa->getFeature(i)->plane()->info);
//			edge->setId(i);
//			optimizer.addEdge(edge);
//		}
//
//		optimizer.initializeOptimization();
//		optimizer.optimize(max_iter_lm);
//
////		std::vector<double> pln_errors;
////		double pln_error_mean=0;
////		for(size_t i=0;i<scan_cur->plane_matches.size();i++)
////		{
////			double tmp=optimizer.activeEdges()[i]->chi2();
////			pln_errors.push_back(tmp);
////			pln_error_mean+=tmp;
////		}
////		pln_error_mean/=pln_errors.size();
////		double pln_error_sigma=0;
////		for(size_t i=0;i<pln_errors.size();i++)
////		{
////			pln_error_sigma+=(pln_errors[i]-pln_error_mean)*(pln_errors[i]-pln_error_mean);
////		}
////		pln_error_sigma/=(pln_errors.size());
////		pln_error_sigma=sqrt(pln_error_sigma);
////		for(size_t i=0;i<pln_errors.size();i++)
////		{
////			if(pln_errors[i]-pln_error_mean>3*pln_error_sigma)
////			{
////				scan_cur->plane_matches[i].cur=0;
////			}
////		}
//
////		if(debug)
////		{
////			fp<<"confirmed planes - "<<pln_error_mean<<", "<<pln_error_sigma<<std::endl;
////			for(size_t i=0;i<scan_cur->plane_matches.size();i++)
////			{
////				fp<<"\t"<<i<<" - "<<pln_errors[i]<<" - "<<scan_cur->plane_matches[i].cur<<std::endl;
////			}
////		}
//
//		optimizer.clear(); 
////		delete linearSolver;
////		delete solver_ptr;
////		delete solver;
//	}



	void PoseEstimation::buildCorrespondence(Scan *scan_cur, Transform Tcr)
	{
		// generate the point cloud to search an NN in;
		// using edge points in ref scan;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		cloud->points.resize(scan_cur->ref()->sizeEdgePoint());
		for(int i=0;i<scan_cur->ref()->sizeEdgePoint();i++)
		{
			cloud->points[i].x=scan_cur->ref()->edgePoint(i)->xyz(0);
			cloud->points[i].y=scan_cur->ref()->edgePoint(i)->xyz(1);
			cloud->points[i].z=scan_cur->ref()->edgePoint(i)->xyz(2);
		}

		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud(cloud);

		// K nearest neighbor search;
		int K=1;
		std::vector<int> pointIdx(K);
		std::vector<float> SquaredDistance(K);
		edge_points_ref.resize(scan_cur->sizeEdgePoint());
		for(int i=0;i<scan_cur->sizeEdgePoint();i++)
		{
			edge_points_ref[i]=0;
			if(!scan_cur->edgePoint(i)->isEdge) continue;
			// one search point in cur scan;
			pcl::PointXYZ searchPoint;
			searchPoint.x=scan_cur->edgePoint(i)->xyz(0);
			searchPoint.y=scan_cur->edgePoint(i)->xyz(1);
			searchPoint.z=scan_cur->edgePoint(i)->xyz(2);
			if(kdtree.nearestKSearch(searchPoint,K,pointIdx,SquaredDistance)>0)
			{
				if(SquaredDistance[0]<0.1) 
				{
					edge_points_ref[i]=scan_cur->ref()->edgePoint(pointIdx[0]);
				}
			}
		}
	}

//	void PoseEstimation::compute_Psi_pi(Scan *scan, Transform Tcr)
//	{
//		if(debug)
//			fp<<"compute_Psi_pi"<<std::endl;
////		Eigen::Matrix<double,6,1> Jpi_xi;
//
//		scan->Psi_pi.setZero();
//		Eigen::Matrix<double,4,6> J_pi;
////		std::ofstream fp_delta;
////		fp_delta.open("delta.txt",std::ios::app);
//		for(size_t i=0;i<scan->plane_matches.size();i++)
//		{
//			if(scan->plane_matches[i].cur==0)
//				continue;
//			Eigen::Vector3d nc=scan->plane_matches[i].cur->normal;
//			Eigen::Vector3d nr=scan->plane_matches[i].ref->normal;
//			Eigen::Matrix<double,1,1> dc;
//			dc(0)=scan->plane_matches[i].cur->d;
//			Eigen::Matrix<double,1,1> dr;
//			dr(0)=scan->plane_matches[i].ref->d;
//
//			J_pi.setZero();
//			J_pi.block<1,3>(3,0)=nr.transpose()*Tcr.R.transpose();
//			J_pi.block<3,3>(0,3)=g2o::skew(Tcr.R*nr);
//			J_pi.block<1,3>(3,3)=-Tcr.t.transpose()*J_pi.block<3,3>(0,3);
//
//			scan->Psi_pi+=J_pi.transpose()*scan->plane_matches[i].cur->cov_inv*J_pi;
//
//
////			Eigen::Matrix3d Hnn=scan->plane_matches[i].computeCov(Tcr).inverse().block<3,3>(0,0);
////			Eigen::Vector3d Hnd=scan->plane_matches[i].computeCov(Tcr).inverse().block<3,1>(0,3);
////			Eigen::Matrix<double,1,1> Hdd=scan->plane_matches[i].computeCov(Tcr).inverse().block<1,1>(3,3);
////
////			Eigen::Matrix3d Cnn=scan->plane_matches[i].ref->cov.block<3,3>(0,0);
////			Eigen::Vector3d Cnd=scan->plane_matches[i].ref->cov.block<3,1>(0,3);
////			Eigen::Matrix<double,1,1> Cdd=scan->plane_matches[i].ref->cov.block<1,1>(3,3);
////
////			Eigen::Vector3d delta_n=nc-Tcr.R*nr;
////			Eigen::Matrix<double,1,1> delta_d=dc-dr+Tcr.t.transpose()*Tcr.R*nr;
////
////			Eigen::Vector3d h_ni=Hnn*delta_n+Hnd*delta_d;
////			Eigen::Matrix<double,1,1> h_di=Hnd.transpose()*delta_n+Hdd*delta_d;
////
////			Eigen::Vector3d delta=Cnn*Tcr.R.transpose()*h_ni+h_di(0,0)*Cnd-h_di(0,0)*Cnn*Tcr.R.transpose()*Tcr.t;
//////			delta=-delta;
//////			delta.normalize();
////			fp_delta<<"n_ref - "<<nr.transpose()<<std::endl;
//////			fp_delta<<"delta - "<<delta.transpose()<<std::endl;
////			Eigen::Matrix<double,1,1> tmp=delta.transpose()*nr;
////			fp_delta<<"delta - "<<delta.norm()<<", "<<acos(tmp(0,0)/delta.norm())*180/M_PI<<std::endl;
////			delta=delta+nr;
////			tmp=delta.transpose()*nr;
//////			delta.normalize();
////			fp_delta<<"nr+delta - "<<delta.norm()<<", "<<acos(tmp(0,0)/delta.norm())*180/M_PI<<std::endl;
////			fp_delta<<"C_cur_inv"<<std::endl<<scan->plane_matches[i].cur->cov_inv<<std::endl;
////			fp_delta<<"delta_n - "<<delta_n.transpose()<<std::endl;
////			fp_delta<<"delta_d - "<<delta_d<<std::endl;
////			fp_delta<<"C_ref"<<std::endl<<scan->plane_matches[i].ref->cov<<std::endl;
////			fp_delta<<"h_n - "<<h_ni.transpose()<<std::endl;
////			fp_delta<<"h_d - "<<h_di<<std::endl;
////			fp_delta<<"t_cr - "<<Tcr.t.transpose()<<std::endl<<std::endl;
////
////
////			Eigen::Matrix3d nr_skew=g2o::skew(nr);
////			Jpi_xi.block<3,1>(0,0)=Tcr.R*nr*h_di;
////			Jpi_xi.block<3,1>(3,0)=nr_skew.transpose()*h_ni+nr_skew*Tcr.t*h_di;
////			scan->Psi_pi+=Jpi_xi*Jpi_xi.transpose();
//		}
////		fp_delta.close();
//		Eigen::EigenSolver<Eigen::MatrixXd> es(scan->Psi_pi);
//		Lambda_pi=es.eigenvalues();
//		Q_pi=es.eigenvectors();
//		if(debug)
//		{
//			fp<<"Lambda_pi - "<<Lambda_pi.transpose()<<std::endl;
////			fpq<<"alpha="<<alpha<<std::endl;
////			fpq<<"Lambda_pi - "<<Lambda_pi.transpose()<<std::endl<<std::endl;;
////			fpd<<"Lambda_pi - "<<Lambda_pi.transpose()<<std::endl;
//			fp<<"Q_pi - "<<std::endl<<Q_pi<<std::endl;
//		}
//		double lambda_pi_1=DBL_MIN;
//		idx_1=-1;
//		for(size_t i=0;i<6;i++)
//		{
//			if(Lambda_pi(i).real()>lambda_pi_1)
//			{
//				lambda_pi_1=Lambda_pi(i).real();
//				idx_1=i;
//			}
//		}
//		sq_lambda_pi_1=sqrt(lambda_pi_1);
//		for(size_t l=0;l<6;l++)
//			q_pi_1(l)=Q_pi(l,idx_1).real();
//	}

//	void PoseEstimation::compute_EdgePoint_weight(Scan *scan, Transform Tcr)
//	{
//		if(debug)
//			fp<<"compute_EdgePoint_weight"<<std::endl;
//		Eigen::Matrix<double,3,6> J_pk;
////		double sq_lambda_pk;
//		for(size_t k=0;k<scan->point_matches.size();k++)
//		{
//			Eigen::Vector3d pc=scan->point_matches[k].cur->xyz;
//			Eigen::Vector3d pr=Tcr.transformPoint(scan->point_matches[k].ref->xyz);
//			Eigen::Matrix3d cov_inv=scan->point_matches[k].cur->cov.inverse();
//
//			J_pk.block<3,3>(0,0).setIdentity();
//			J_pk.block<3,3>(0,3)=g2o::skew(Tcr.R*pr);
//			scan->point_matches[k].Psi_pk=J_pk.transpose()*cov_inv*J_pk;
//
//			Eigen::EigenSolver<Eigen::MatrixXd> es(scan->point_matches[k].Psi_pk);
//			Eigen::Matrix<std::complex<double>,6,1> Lambda_pk=es.eigenvalues();
//			Eigen::Matrix<std::complex<double>,6,6> Q_pk=es.eigenvectors();
//			double lambda_pk=DBL_MIN;
//			int idx=-1;
//			for(size_t i=0;i<6;i++)
//			{
//				if(Lambda_pk(i).real()>lambda_pk)
//				{
//					lambda_pk=Lambda_pk(i).real();
//					idx=i;
//				}
//			}
//			scan->point_matches[k].sq_lambda_pk=sqrt(lambda_pk);
////			scan->point_matches[k].v_pk;
//			for(size_t l=0;l<6;l++)
//				scan->point_matches[k].v_pk(l)=Q_pk(l,idx).real();
//
//			if(!useWeight)
//			{
//				scan->point_matches[k].weight=1;
//				continue;
//			}
//
////			scan->point_matches[k].gradient.block<3,1>(0,0)=-cov_inv*(pc-pr);
////			Eigen::Vector3d pt_ref=Tcr.R*scan->point_matches[k].ref->xyz;
////			scan->point_matches[k].gradient.block<3,1>(3,0)=pt_ref.cross(scan->point_matches[k].gradient.block<3,1>(0,0));
//
////			if(scan->point_matches[k].gradient.norm()==0)
////			{
////				scan->point_matches[k].weight=0;
////				continue;
////			}
////			Eigen::Matrix<double,6,1> v_pk=scan->point_matches[k].gradient;
////			v_pk.normalize();
//			scan->point_matches[k].weight=0;
//			for(size_t l=0;l<6;l++)
//			{
//				double lambda_l=Lambda_pi(l).real();
//				Eigen::Matrix<double,6,1> q_l;
//				for(size_t j=0;j<6;j++)
//				{
//					q_l(j)=Q_pi(j,l).real();
//				}
//				scan->point_matches[k].weight+=fabs(q_l.dot(scan->point_matches[k].v_pk))/exp(alpha*sqrt(fabs(lambda_l))/sq_lambda_pi_1);
//			}
//			scan->point_matches[k].weight/=6;
//		}
//		Eigen::Matrix<double,6,1> Weight_p_sum;
//		Weight_p_sum.setZero();
//		for(size_t k=0;k<scan->point_matches.size();k++)
//		{
//			Weight_p_sum+=scan->point_matches[k].weight*scan->point_matches[k].sq_lambda_pk*scan->point_matches[k].v_pk;
//		}
//		Weight_p=beta*sq_lambda_pi_1/Weight_p_sum.norm();
//		if(debug)
//		{
//			fp<<"point matches - "<<scan->point_matches.size()<<std::endl;
//			fp<<"Weight_p_sum - "<<Weight_p_sum.transpose()<<std::endl;
//			fp<<"Weight_p_sum.norm - "<<Weight_p_sum.norm()<<std::endl;
//			Weight_p_sum.normalize();
//			fp<<"Weight_p_sum.normalize - "<<Weight_p_sum.transpose()<<std::endl;
//			for(size_t l=0;l<6;l++)
//			{
//				double lambda_l=Lambda_pi(l).real();
//				Eigen::Matrix<double,6,1> q_l;
//				for(size_t j=0;j<6;j++)
//				{
//					q_l(j)=Q_pi(j,l).real();
//				}
//				fp<<Weight_p_sum.dot(q_l)<<"\t";
//			}
//			fp<<std::endl;
//			fp<<"Weight_p - "<<Weight_p<<std::endl;
//			fpq<<"======================================================="<<std::endl;
//			for(size_t i=0;i<scan->point_matches.size();i++)
//			{
//				fpq<<i<<"\t"<<scan->point_matches[i].gradient.transpose()<<std::endl;;
////				fp<<"\t"<<i<<" - "<<scan->point_matches[i].weight<<"\t";
//				fpd<<"\t"<<i<<"\t"<<scan->point_matches[i].weight<<"\t";
//				fpd<<scan->point_matches[i].gradient.transpose()<<"\t";
//				for(size_t l=0;l<6;l++)
//				{
//					double lambda_l=Lambda_pi(l).real();
//					Eigen::Matrix<double,6,1> q_l;
//					for(size_t j=0;j<6;j++)
//					{
//						q_l(j)=Q_pi(j,l).real();
//					}
//					scan->point_matches[i].gradient.normalize();
//					fpd<<"\t"<<scan->point_matches[i].gradient.dot(q_l);
//				}
////				fp<<std::endl;
//				fpd<<std::endl;
//			}
//		}
//	}
}


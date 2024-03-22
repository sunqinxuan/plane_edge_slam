/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-06-07 16:36
#
# Filename:		motion_estimation.cpp
#
# Description: 
#
===============================================*/

#include "motion/motion_estimation.h"

namespace ulysses
{
	using namespace std;
	PlaneLineBA::PlaneLineBA(Map *map)
	{
		// parameters_ - cameras
		num_cameras_=0;
		std::map<double,int> camera_indices; int counter=0;
		for(const_iterCamera it=map->beginCamera();it!=map->endCamera();it++)
		{
			Transform T=it->second;
			cout<<it->first<<", "<<it->second<<endl;
			Eigen::Quaterniond quat=T.Quat();
			parameters_.push_back(quat.w());
			parameters_.push_back(quat.x());
			parameters_.push_back(quat.y());
			parameters_.push_back(quat.z());
			parameters_.push_back(T.t(0));
			parameters_.push_back(T.t(1));
			parameters_.push_back(T.t(2));
			timestamps_.push_back(it->first);
			camera_indices[it->first]=counter++;
			num_cameras_++;
		}

		// parameters_ - features;
		num_planes_=0; num_lines_=0; counter=0;
		num_observ_planes_=0; num_observ_lines_=0;
		for(const_iterLandmark it=map->beginLandmark();it!=map->endLandmark();it++)
		{
			Landmark* lm=it->second;
			cout<<lm<<endl;
			if(lm->Type()==PLANE)
			{
				PlaneLM* p=lm->planelm();
				parameters_.push_back(p->pi(0));
				parameters_.push_back(p->pi(1));
				parameters_.push_back(p->pi(2));
				parameters_.push_back(p->pi(3));
				num_planes_++;
			}
			else if(lm->Type()==LINE)
			{
				LineLM* l=lm->linelm();
				parameters_.push_back(l->L(0));
				parameters_.push_back(l->L(1));
				parameters_.push_back(l->L(2));
				parameters_.push_back(l->L(3));
				parameters_.push_back(l->L(4));
				parameters_.push_back(l->L(5));
				num_lines_++;
			}
			features_id_.push_back(it->first);
			for(iterObserv it_obs=lm->beginObserv();it_obs!=lm->endObserv();it_obs++)
			{
				Scan* s=map->scan(it_obs->first);
				for(int j=0;j<it_obs->second.size();j++)
				{
					Feature* f=s->findFeature(it_obs->second[j]);
					if(f->Type()==PLANE)
					{
						observations_.push_back(f->plane()->n(0));
						observations_.push_back(f->plane()->n(1));
						observations_.push_back(f->plane()->n(2));
						observations_.push_back(f->plane()->d);
						for(int i=0;i<16;i++) sqrt_information_.push_back(f->plane()->sqrt_info.data()[i]);
						num_observ_planes_++;
						
					}
					else if(f->Type()==LINE)
					{
						observations_.push_back(f->line()->u(0));
						observations_.push_back(f->line()->u(1));
						observations_.push_back(f->line()->u(2));
						observations_.push_back(f->line()->v(0));
						observations_.push_back(f->line()->v(1));
						observations_.push_back(f->line()->v(2));
						for(int i=0;i<36;i++) sqrt_information_.push_back(f->line()->sqrt_info.data()[i]);
						num_observ_lines_++;
					}
					camera_index_.push_back(camera_indices.find(it_obs->first)->second);
					feature_index_.push_back(counter);
				}
			}
			counter++;
		}
		num_observations_=num_observ_planes_+num_observ_lines_;
		num_parameters_=7*num_cameras_+4*num_planes_+6*num_lines_;
	}

	PlaneLineBA::~PlaneLineBA() {}

//	void PlaneLineBA::addEdgePoints(Scan *scan, const std::vector<EdgePoint*> &edge_points_ref)
//	{
//		num_edge_=0; num_observ_edge_=0;
//		for(size_t i=0;i<edge_points_ref.size();i++)
//		{
//			if(edge_points_ref[i]==0) continue;
//
////			measure_pt.pt_cur=scan_cur->edgePoint(i)->xyz;
////			measure_pt.pt_ref=edge_points_ref[i]->xyz;
//		}
//	}

//	bool PlaneErrorCostFunction::Evaluate(double const* const* parameters, double* residuals, double** jacobian) const
//	{
//		std::ofstream fp;
//		fp.open("plane_error.txt",std::ios::app);
//
//		Eigen::Quaterniond q_cr(parameters[1][0],parameters[1][1],parameters[1][2],parameters[1][3]);
//		Eigen::Vector3d t_cr(parameters[1][4],parameters[1][5],parameters[1][6]);
////		Eigen::Map<const Eigen::Vector4d> pi(parameters[1]);
//		Eigen::Map<Eigen::Vector4d> residual(residuals);
//		fp<<"q_cr "<<q_cr.w()<<"\t"<<q_cr.vec().transpose()<<std::endl;
//		fp<<"t_cr "<<t_cr.transpose()<<std::endl;
////		fp<<"pi "<<pi.transpose()<<std::endl;
//
//		Eigen::Matrix3d R_cr=q_cr.toRotationMatrix();
//		Eigen::Matrix4d T_cr; // transform for planes;
//		T_cr.setIdentity();
//		T_cr.block<3,3>(0,0)=R_cr;
//		T_cr.block<1,3>(3,0)=-t_cr.transpose()*R_cr;
//
////		Eigen::Vector4d nd;
////		double n_norm=pi.block<3,1>(0,0).norm();
////		nd=pi/n_norm;
//
//		residual = observed_pi_cur_-T_cr*observed_pi_ref_;
//		residual.applyOnTheLeft(sqrt_information_);
//		fp<<"T_cr"<<std::endl<<T_cr<<std::endl;
//		fp<<"sqrt_information_"<<std::endl<<sqrt_information_<<std::endl;
//		fp<<"observed_pi_ref_ "<<observed_pi_ref_.transpose()<<std::endl;
//		fp<<"observed_pi_cur_ "<<observed_pi_cur_.transpose()<<std::endl;
//		fp<<"residual "<<residual.transpose()<<std::endl;
//
//		if(jacobian)
//		{
//			fp<<"jacobian!=NULL"<<std::endl;
//			for(size_t i=0;i<28;i++) jacobian[0][i]=0.1;
//			for(size_t i=0;i<28;i++) jacobian[1][i]=0.1;
//		}
//
//		return true;
//	}



//
//	bool LineLocalParameterization::Plus(const double *x, const double *delta, double *x_plus_delta) const
//	{
//		Eigen::Map<const Eigen::Vector3d> u(x);
//		Eigen::Map<const Eigen::Vector3d> v(x+3);
//		Eigen::Map<const Eigen::Vector3d> vec_theta(delta);
//		const double theta=delta[3];
//
//		Eigen::Matrix3d U;
//		Eigen::Matrix2d W;
//		Eigen::Vector3d uxv=u.cross(v);
//		double sigma_1=u.norm();
//		double sigma_2=v.norm();
//		double sigma_norm=sqrt(sigma_1*sigma_1+sigma_2*sigma_2);
//
//		U.block<3,1>(0,0)=u/sigma_1;
//		U.block<3,1>(0,1)=v/sigma_2;
//		U.block<3,1>(0,2)=uxv/uxv.norm();
//		W(0,0)=sigma_1/sigma_norm;
//		W(0,1)=-sigma_2/sigma_norm;
//		W(1,0)=sigma_2/sigma_norm;
//		W(1,1)=sigma_1/sigma_norm;
//
//		Eigen::Matrix3d R_vec_theta;
//		Eigen::Matrix2d R_theta;
//		Eigen::AngleAxisd angle_axis(vec_theta.norm(),vec_theta/vec_theta.norm());
//		R_vec_theta=angle_axis.toRotationMatrix();
//		if(vec_theta.norm()<1e-7)
//		{
//			R_vec_theta.setIdentity();
//		}
//		R_theta(0,0)=cos(theta);
//		R_theta(0,1)=-sin(theta);
//		R_theta(1,0)=sin(theta);
//		R_theta(1,1)=cos(theta);
//
//		U.applyOnTheRight(R_vec_theta);
//		W.applyOnTheRight(R_theta);
//		double w11=W(0,0);
//		double w21=W(1,0);
//		Eigen::Vector3d u1=U.block<3,1>(0,0);
//		Eigen::Vector3d u2=U.block<3,1>(0,1);
//		Eigen::Vector3d u3=U.block<3,1>(0,2);
//
//		Eigen::Map<Eigen::Vector3d> u_update(x_plus_delta);
//		Eigen::Map<Eigen::Vector3d> v_update(x_plus_delta+3);
//		u_update=w11*u1;
//		v_update=w21*u2;
//		
//		return true;
//	}
//
//	bool LineLocalParameterization::ComputeJacobian(const double *x, double *jacobian) const
//	{
//		Eigen::Map<const Eigen::Vector3d> u(x);
//		Eigen::Map<const Eigen::Vector3d> v(x+3);
//		Eigen::Map<Eigen::Matrix<double,6,4,Eigen::RowMajor> > J(jacobian);
//
//		double w11=u.norm();
//		double w21=v.norm();
//		Eigen::Vector3d u1=u/w11;
//		Eigen::Vector3d u2=v/w21;
//		Eigen::Vector3d u3=u1.cross(u2);
//
//		J.setZero();
//		J.block<3,1>(3,0)=w21*u3;
//		J.block<3,1>(0,1)=-w11*u3;
//		J.block<3,1>(0,2)=w11*u2;
//		J.block<3,1>(3,2)=-w21*u1;
//		J.block<3,1>(0,3)=-w21*u1;
//		J.block<3,1>(3,3)=w11*u2;
//
//		return true;
//	}

	bool MotionEstimation::estimate(Scan *scan, boost::shared_ptr<pcl::visualization::PCLVisualizer> vis)
	{
//		debug=true;
		if(debug) 
		{
			fp.open("motion_estimation.txt",std::ios::app);
			fp<<std::endl<<"estimate*******************************************"<<std::endl;
		}

		int iterations = 0;
		bool converged = false;
		double chi2_pre=DBL_MAX, chi2=DBL_MAX;

//					scan->vis2Scans(vis,IDENTITY);

		while(iterations<max_iter_icp && !converged)
		{
			if(debug)
			{
				fp<<std::endl<<"========================================"<<std::endl;
				fp<<"icp iteration - "<<iterations<<std::endl;
				fp<<"Tcr = "<<scan->Tcr()<<std::endl;
			}

			buildCorrespondence(scan);
//			cout<<"edge_points_ref : "<<edge_points_ref.size()<<endl;
//
			chi2_pre=chi2;
//			cout<<"chi2= "<<chi2<<endl;

//			map=new Map;
//			map->addScan(scan);
//			cout<<"camera "<<map->sizeCamera()<<endl;
//			cout<<"landmarks "<<map->sizeLandmark()<<endl;
			chi2=optimize(scan);
//			scan->Tcr()=map->camera(scan->time())*map->camera(scan->ref()->time()).inv();
			scan->localize();
//			std::cout<<"Tcr = "<<scan->Tcr()<<std::endl;
//			delete map;

//				cout<<"icp iteration - "<<iterations<<std::endl;
//					scan->vis2Scans(vis,ESTIMATE);

			iterations++;
			if(chi2>=chi2_pre)
				converged=true;
		}

		if(debug) fp.close();
		return true;
	}

	void MotionEstimation::buildCorrespondence(Scan *scan_cur)
	{
		if(scan_cur->ref()->sizeEdgePoint()==0 || scan_cur->sizeEdgePoint()==0) return;
		// generate the point cloud to search an NN in;
		// using edge points in ref scan;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		cloud->points.resize(scan_cur->ref()->sizeEdgePoint());
		for(int i=0;i<scan_cur->ref()->sizeEdgePoint();i++)
		{
//			Eigen::Vector3d pr=Tcr.transformPoint(scan_ref->edge_points[i]->xyz);
			cloud->points[i].x=scan_cur->ref()->edgePoint(i)->xyz(0);
			cloud->points[i].y=scan_cur->ref()->edgePoint(i)->xyz(1);
			cloud->points[i].z=scan_cur->ref()->edgePoint(i)->xyz(2);
		}
		pcl::transformPointCloud(*cloud,*cloud,scan_cur->Tcr().getMatrix4f());

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

	double MotionEstimation::optimize(Scan *scan)
	{
		if(debug) fp<<"optimize: *******************************************"<<std::endl;

		ceres::Problem *problem=new ceres::Problem;

		// LocalParameterization for cameras;
		ceres::LocalParameterization* camera_parameterization=
			new ceres::ProductParameterization(new ceres::QuaternionParameterization(),new ceres::IdentityParameterization(3));
		double *camera_ref=new double[7]{1,0,0,0,0,0,0};
		problem->AddParameterBlock(camera_ref,7,camera_parameterization);
		problem->SetParameterBlockConstant(camera_ref);
		Eigen::Quaterniond q=scan->Tcr().Quat();
		Eigen::Vector3d t=scan->Tcr().t;
		double *camera_cur=new double[7]{q.w(),q.x(),q.y(),q.z(),t(0),t(1),t(2)};
		problem->AddParameterBlock(camera_cur,7,camera_parameterization);

//		const int camera_block_size=plane_line_ba->camera_block_size();
//		const int plane_block_size=plane_line_ba->plane_block_size();
//		const int line_block_size=plane_line_ba->line_block_size();
//		double* cameras=plane_line_ba->mutable_cameras();
//		double* planes=plane_line_ba->mutable_planes();
//		double* lines=plane_line_ba->mutable_lines();
//
//
//		double* observations=plane_line_ba->observations();
//		double* sqrt_information=plane_line_ba->sqrt_information();
//		if(debug) fp<<"plane observations"<<endl;

		FeatureAssociation *fa=scan->association();

		if(debug) fp<<"plane observations"<<endl;
		if(use_plane_)
		for(size_t i=0;i<fa->size();i++)
		{
			if(fa->getFeature(i)->Type()!=PLANE) continue;

			Eigen::Vector3d nc=fa->getFeature(i)->getDirection();
			double dc=fa->getFeature(i)->getDistance().norm();
			Eigen::Vector3d nr=fa->getFeatureRef(i)->getDirection();
			double dr=fa->getFeatureRef(i)->getDistance().norm();

			// each residual block takes a plane/line and a camera as input
			// and outputs a 4/6 dimensional residual;
			Eigen::Vector4d pi_cur,pi_ref;
			pi_cur.block<3,1>(0,0)=nc; pi_cur(3)=dc;
			pi_ref.block<3,1>(0,0)=nr; pi_ref(3)=dr;
//			observations+=plane_block_size;
			Eigen::Matrix4d information;
			information=fa->getFeature(i)->plane()->sqrt_info;
//			sqrt_information+=plane_block_size*plane_block_size;

			if(debug)
			{
				fp<<i<<endl;
				fp<<"pi_cur= "<<pi_cur.transpose()<<endl;
				fp<<"pi_ref= "<<pi_ref.transpose()<<endl;
				fp<<"sqrt_info="<<endl<<information<<endl;
//				fp<<"camera idx="<<plane_line_ba->camera_index(i)<<endl;
//				for(int j=0;j<camera_block_size;j++) fp<<camera[j]<<" "; fp<<endl;
//				fp<<"feature idx="<<plane_line_ba->feature_index(i)<<endl;
//				for(int j=0;j<plane_block_size;j++) fp<<plane[j]<<" "; fp<<endl;
			}

//			information.setIdentity();
			ceres::CostFunction* cost_function=PlaneError::Create(pi_ref,pi_cur,information);
			ceres::LossFunction* loss_function=NULL; //new ceres::HuberLoss(1.0);
			problem->AddResidualBlock(cost_function,loss_function,camera_ref,camera_cur);
		}

		if(debug) fp<<"edge observations"<<endl;
		if(use_edge_)
		for(size_t i=0;i<edge_points_ref.size();i++)
		{
			if(edge_points_ref[i]==0) continue;

			// each residual block takes a plane/line and a camera as input
			// and outputs a 4/6 dimensional residual;
			Eigen::Vector3d pi_cur=scan->edgePoint(i)->xyz;
			Eigen::Vector3d pi_ref=edge_points_ref[i]->xyz;
//			Vector6d L(observations);
//			observations+=line_block_size;
			Eigen::Matrix3d information(scan->edgePoint(i)->sqrt_info);
//			sqrt_information+=line_block_size*line_block_size;

//			double* camera=cameras+camera_block_size*plane_line_ba->camera_index(i+plane_line_ba->num_observ_planes());
//			double* line=lines+line_block_size*(plane_line_ba->feature_index(i+plane_line_ba->num_observ_planes())-plane_line_ba->num_planes());

			if(debug)
			{
				fp<<i<<endl;
				fp<<"pi_cur= "<<pi_cur.transpose()<<endl;
				fp<<"pi_ref= "<<pi_ref.transpose()<<endl;
				fp<<"sqrt_info="<<endl<<information<<endl;
//				fp<<"camera idx="<<plane_line_ba->camera_index(i)<<endl;
//				for(int j=0;j<camera_block_size;j++) fp<<camera[j]<<" "; fp<<endl;
//				fp<<"feature idx="<<plane_line_ba->feature_index(i)<<endl;
//				for(int j=0;j<plane_block_size;j++) fp<<plane[j]<<" "; fp<<endl;
			}

//			information.setIdentity();
			ceres::CostFunction* cost_function;
			cost_function=EdgeError::Create(pi_ref,pi_cur,information);
			ceres::LossFunction* loss_function=NULL; //new ceres::HuberLoss(1.0);
			problem->AddResidualBlock(cost_function,loss_function,camera_ref,camera_cur);
		}

		ceres::Solver::Options options;
//		options.minimizer_progress_to_stdout=true;
		options.max_num_iterations=max_iter_lm;
		options.linear_solver_type=ceres::DENSE_SCHUR;
//		options.trust_region_strategy_type=ceres::DOGLEG;
//		options.gradient_tolerance=1e-16;
//		options.function_tolerance=1e-16;
		ceres::Solver::Summary summary;

		Solve(options,problem,&summary);
		
		Eigen::Quaterniond q_cr(camera_cur[0],camera_cur[1],camera_cur[2],camera_cur[3]);
		q_cr.normalize();
		Eigen::Vector3d t_cr(camera_cur[4],camera_cur[5],camera_cur[6]);
		scan->Tcr()=Transform(q_cr,t_cr);

//		std::cout<<"Tcr = "<<scan->Tcr()<<std::endl;
//		scan_cur->localize();

//
//		// LocalParameterization for planes;
//		ceres::LocalParameterization* plane_parameterization=new ceres::QuaternionParameterization();
//		for(size_t i=0;i<plane_line_ba->num_planes();i++)
//		{
//			problem->AddParameterBlock(planes+plane_block_size*i,plane_block_size,plane_parameterization);
//			if(feature_constant) problem->SetParameterBlockConstant(planes+plane_block_size*i);
//		}
//
//		// LocalParameterization for lines;
//		ceres::LocalParameterization* line_parameterization=new LineLocalParameterization();
//		for(size_t i=0;i<plane_line_ba->num_lines();i++)
//		{
//			problem->AddParameterBlock(lines+line_block_size*i,line_block_size,line_parameterization);
//			if(feature_constant) problem->SetParameterBlockConstant(lines+line_block_size*i);
//		}
		delete problem;
//		delete camera_parameterization;
		delete [] camera_ref;
		delete [] camera_cur;

		return summary.final_cost;
	}

//	double MotionEstimation::optimize(Map *map)
//	{
//		PlaneLineBA *plane_line_ba=new PlaneLineBA(map);//,num_frames,use_plane_,use_line_,use_shadow_);
//		ceres::Problem problem;
//		buildProblem(plane_line_ba,&problem);
//
//		ceres::Solver::Options options;
//		options.minimizer_progress_to_stdout=true;
//		options.max_num_iterations=max_iter_lm;
//		options.linear_solver_type=ceres::DENSE_SCHUR;
////		options.trust_region_strategy_type=ceres::DOGLEG;
////		options.gradient_tolerance=1e-16;
////		options.function_tolerance=1e-16;
//		ceres::Solver::Summary summary;
//		Solve(options,&problem,&summary);
////		std::cout<<summary.FullReport()<<"\n";
//		
//		/* the followings are updates of the map */
//		/*****************************************/
//		const int camera_block_size=plane_line_ba->camera_block_size();
//		double* cameras=plane_line_ba->mutable_cameras();
//		if(debug) fp<<"\nBA cameras\n";
//		for(size_t i=0;i<plane_line_ba->num_cameras();i++)
//		{
//			double* camera=cameras+camera_block_size*i;
//			if(debug) fp<<"\t"<<camera[0]<<"\t"<<camera[1]<<"\t"<<camera[2]<<"\t"<<camera[3]
//						  <<"\t"<<camera[4]<<"\t"<<camera[5]<<"\t"<<camera[6]<<std::endl;
//			Eigen::Quaterniond quat(camera[0],camera[1],camera[2],camera[3]);
//			Eigen::Vector3d t_tmp(camera[4],camera[5],camera[6]);
////			map->cameras[idx_camera_initial+i]=Transform(quat,t_tmp);
//			double t=plane_line_ba->timestamps(i);
//			map->updateCamera(t,Transform(quat,t_tmp));
//		}
////		Tcr=map->cameras[map->cameras.size()-1]*map->cameras[map->cameras.size()-2].inv();
//		/*****************************************/
//
//		const int plane_block_size=plane_line_ba->plane_block_size();
//		const int line_block_size=plane_line_ba->line_block_size();
//
//		double* planes=plane_line_ba->mutable_planes();
//		if(debug) fp<<"\nBA planes\n";
//		for(size_t i=0;i<plane_line_ba->num_planes();i++)
//		{
//			double* plane=planes+plane_block_size*i;
//			std::string id=plane_line_ba->features_id(i);
//			Landmark* lm=map->findLandmark(id);
//			memcpy(lm->planelm()->pi.data(),plane,plane_block_size*sizeof(double));
//			if(debug) fp<<"\t"<<plane[0]<<"\t"<<plane[1]<<"\t"<<plane[2]<<"\t"<<plane[3]<<std::endl;
//		}
//
//		double* lines=plane_line_ba->mutable_lines();
//		if(debug) fp<<"\nBA lines\n";
//		for(size_t i=0;i<plane_line_ba->num_lines();i++)
//		{
//			double* line=lines+line_block_size*i;
//			std::string id=plane_line_ba->features_id(i+plane_line_ba->num_planes());
//			Landmark* lm=map->findLandmark(id);
//			memcpy(lm->linelm()->L.data(),line,line_block_size*sizeof(double));
//			if(debug) fp<<"\t"<<line[0]<<"\t"<<line[1]<<"\t"<<line[2]<<"\t"<<line[3]<<"\t"<<line[4]<<"\t"<<line[5]<<std::endl;
//		}
//
//		delete plane_line_ba;
//		return summary.final_cost;
//	}


//	void MotionEstimation::buildProblem(PlaneLineBA* plane_line_ba, ceres::Problem* problem)
//	{
//		if(debug) fp<<"buildProblem: *******************************************"<<std::endl;
//
//		const int camera_block_size=plane_line_ba->camera_block_size();
//		const int plane_block_size=plane_line_ba->plane_block_size();
//		const int line_block_size=plane_line_ba->line_block_size();
//		double* cameras=plane_line_ba->mutable_cameras();
//		double* planes=plane_line_ba->mutable_planes();
//		double* lines=plane_line_ba->mutable_lines();
//
//		double* observations=plane_line_ba->observations();
//		double* sqrt_information=plane_line_ba->sqrt_information();
//		if(debug) fp<<"plane observations"<<endl;
//		for(size_t i=0;i<plane_line_ba->num_observ_planes();i++)
//		{
//			// each residual block takes a plane/line and a camera as input
//			// and outputs a 4/6 dimensional residual;
//			Eigen::Vector4d pi(observations);
//			observations+=plane_block_size;
//			Eigen::Matrix4d information(sqrt_information);
//			sqrt_information+=plane_block_size*plane_block_size;
//
//			double* camera=cameras+camera_block_size*plane_line_ba->camera_index(i);
//			double* plane=planes+plane_block_size*plane_line_ba->feature_index(i);
//
//			if(debug)
//			{
//				fp<<i<<endl;
//				fp<<pi.transpose()<<endl;
//				fp<<"camera idx="<<plane_line_ba->camera_index(i)<<endl;
//				for(int j=0;j<camera_block_size;j++) fp<<camera[j]<<" "; fp<<endl;
//				fp<<"feature idx="<<plane_line_ba->feature_index(i)<<endl;
//				for(int j=0;j<plane_block_size;j++) fp<<plane[j]<<" "; fp<<endl;
//			}
//
////			information.setIdentity();
//			ceres::CostFunction* cost_function=PlaneError::Create(pi,information);
//			ceres::LossFunction* loss_function=NULL; //new ceres::HuberLoss(1.0);
//			problem->AddResidualBlock(cost_function,loss_function,camera,plane);
//		}
//
//		if(debug) fp<<"line observations"<<endl;
//		for(size_t i=0;i<plane_line_ba->num_observ_lines();i++)
//		{
//			// each residual block takes a plane/line and a camera as input
//			// and outputs a 4/6 dimensional residual;
//			Vector6d L(observations);
//			observations+=line_block_size;
//			Matrix6d information(sqrt_information);
//			sqrt_information+=line_block_size*line_block_size;
//
//			double* camera=cameras+camera_block_size*plane_line_ba->camera_index(i+plane_line_ba->num_observ_planes());
//			double* line=lines+line_block_size*(plane_line_ba->feature_index(i+plane_line_ba->num_observ_planes())-plane_line_ba->num_planes());
//
//			if(debug)
//			{
//				fp<<i<<endl;
//				fp<<L.transpose()<<endl;
//				fp<<"camera idx="<<plane_line_ba->camera_index(i+plane_line_ba->num_observ_planes())<<endl;
//				for(int j=0;j<camera_block_size;j++) fp<<camera[j]<<" "; fp<<endl;
//				fp<<"feature idx="<<plane_line_ba->feature_index(i+plane_line_ba->num_observ_planes())<<endl;
//				for(int j=0;j<line_block_size;j++) fp<<line[j]<<" "; fp<<endl;
//			}
//
////			information.setIdentity();
//			ceres::CostFunction* cost_function;
//			cost_function=LineError::Create(L,information);
//			ceres::LossFunction* loss_function=NULL; //new ceres::HuberLoss(1.0);
//			problem->AddResidualBlock(cost_function,loss_function,camera,line);
//		}
//
//		// LocalParameterization for cameras;
//		ceres::LocalParameterization* camera_parameterization=
//			new ceres::ProductParameterization(new ceres::QuaternionParameterization(),new ceres::IdentityParameterization(3));
//		for(size_t i=0;i<plane_line_ba->num_cameras();i++)
//		{
//			problem->AddParameterBlock(cameras+camera_block_size*i,camera_block_size,camera_parameterization);
//			if(i==0) problem->SetParameterBlockConstant(cameras+camera_block_size*i);
//		}
//
//		// LocalParameterization for planes;
//		ceres::LocalParameterization* plane_parameterization=new ceres::QuaternionParameterization();
//		for(size_t i=0;i<plane_line_ba->num_planes();i++)
//		{
//			problem->AddParameterBlock(planes+plane_block_size*i,plane_block_size,plane_parameterization);
//			if(feature_constant) problem->SetParameterBlockConstant(planes+plane_block_size*i);
//		}
//
//		// LocalParameterization for lines;
//		ceres::LocalParameterization* line_parameterization=new LineLocalParameterization();
//		for(size_t i=0;i<plane_line_ba->num_lines();i++)
//		{
//			problem->AddParameterBlock(lines+line_block_size*i,line_block_size,line_parameterization);
//			if(feature_constant) problem->SetParameterBlockConstant(lines+line_block_size*i);
//		}
//	}

}


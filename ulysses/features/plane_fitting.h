/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-05-19 13:10
#
# Filename:		plane_fitting.h
#
# Description: 
#
===============================================*/

#ifndef _PLANE_FITTING_H_
#define _PLANE_FITTING_H_

#include "types/types.h"
#include "types/map.h"

namespace ulysses
{
	extern CameraIntrinsic camera_intrinsic;

	class PlaneFitting
	{
	public:

		PlaneFitting(const std::string &settingFile)
		{
			remove("plane_fitting.txt");
			cv::FileStorage settings(settingFile,cv::FileStorage::READ);
			settings["debug"]>>debug;
			settings["PlaneFitting.method"]>>fitting_method;
			settings.release();
		}
		~PlaneFitting() {}

		void setDebug(bool d) {debug=d;}
		void setFittingMethod(std::string m) {fitting_method=m;}

		void fitPlanes(Scan *scan);

		void fitPlaneModel(Plane *plane);

		void removeOutliers(Plane *plane);

	private:
		
		bool debug;
		std::ofstream fp;
		std::string fitting_method;

		void computePointWeight(Plane *plane, Eigen::Vector3d n);
		void computeCentroidScatter(Plane *plane, bool weighted=false);
		void computePlaneInfo(Plane *plane, bool weighted=false);

		void EigenSolve(const Eigen::Matrix3d &scatter, Eigen::Vector3d &n);
	};
}

#endif

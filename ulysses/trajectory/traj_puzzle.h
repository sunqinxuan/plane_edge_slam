/*==============================================
#
# Author: Sun Qinxuan
#
# Email: sunqinxuan@outlook.com
#
# Last modified:	2020-06-04 10:44
#
# Filename:		traj_puzzle.h
#
# Description: 
#
===============================================*/
#pragma once
#include <regex>
#include "types/types.h"
#include "types/map.h"

namespace ulysses
{
	struct TrajFile
	{
		TrajFile(std::string path, std::string file)
		{
			file_name=path+"/"+file;
			fp.open(file_name,std::ios::in);
			fp>>start_time;
			fp.close();
		}

		std::string file_name;
		std::ifstream fp;
		double start_time;

		bool operator < (const TrajFile &m) const
		{
			return start_time < m.start_time;
		}
	};
	
	class TrajPuzzle
	{
	public:

		TrajPuzzle() {}

		~TrajPuzzle() {}

//		Transform Traj(double t) {return trajectory.find(t)->second;}
//		size_t TrajLength() {return trajectory.size();}

		void readTrajFiles(const std::string traj_path);

//		void readTraj2Map(int n);

	private:

		std::ofstream fp;

//		const std::string traj_path;
		std::vector<TrajFile> traj_file_list;
//		std::vector<Transform> trajectory;
		std::map<double,Transform> trajectory;
	};
}

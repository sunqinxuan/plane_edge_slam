#!/bin/sh

 seq_path="/home/sun/rgbd_dataset_freiburg3_cabinet"
 #seq_path="/media/sun/Elements/sun/rgbd_dataset_freiburg3_cabinet"
 #seq_path="/home/sun/dataset/data0804/scanning_3/"
 
 PROJECT_SOURCE_DIR="/home/sun/shadow_SLAM"
 setting_path=${PROJECT_SOURCE_DIR}"/run/settings/"
 setting_file="settings_fr3.yaml"
 calib_file_mocap="calib_sensor_marker_0724_static.yaml"
 calib_file_depth="calib_pose.yaml"
 #echo "setting_file=" ${setting_file}
 
 # create the association file
 ${PROJECT_SOURCE_DIR}/run/evaluate/associate.py ${seq_path}/depth.txt ${seq_path}/rgb.txt --max_difference 0.05
 #${PROJECT_SOURCE_DIR}/run/evaluate/associate.py ${seq_path}/depth_hd.txt ${seq_path}/rgb_hd.txt
 ${PROJECT_SOURCE_DIR}/run/evaluate/associate.py association.txt ${seq_path}/groundtruth.txt --max_difference 0.05
 #${PROJECT_SOURCE_DIR}/run/evaluate/associate.py association.txt ${seq_path}/outputCalib.txt --max_difference 0.05
 cp association.txt ${seq_path}/association.txt
 
 # main system
 bin/PlaneLineSLAM ${seq_path} ${setting_path} ${setting_file} ${calib_file_mocap} ${calib_file_depth}

 exit

 RETURN=$?
 echo "RETURN=" ${RETURN}
 
 if [ ${RETURN} -ne 0 ]
 	then 
 		echo "not equal"
 		exit
 fi
 
 # save the results
 time=$(date "+%Y%m%d%H%M%S")
 mkdir ${time}
 
 # evaluate rpe
 ${PROJECT_SOURCE_DIR}/run/evaluate/evaluate_rpe.py ${seq_path}/outputCalib.txt ${time}/traj.txt --fixed_delta  --delta 1 --delta_unit s --plot ${time}/rpe.png --offset 0 --scale 1 --save ${time}/rpe.txt --verbose
 
 # evaluate ate
 ${PROJECT_SOURCE_DIR}/run/evaluate/evaluate_ate.py ${seq_path}/outputCalib.txt ${time}/traj.txt --plot ${time}/ate.png --offset 0 --scale 1 --verbose
 

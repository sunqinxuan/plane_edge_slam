#!/bin/sh

 #seq_path="/media/sun/Elements/sun/rgbd_dataset_freiburg3_cabinet"
 seq_path="/media/sun/Elements/sun/rgbd_dataset_freiburg3_cabinet"
 
 PROJECT_SOURCE_DIR="/home/sun/shadow_SLAM"
 setting_path=${PROJECT_SOURCE_DIR}"/run/settings/"
 setting_file="settings_kinect2.yaml"
 calib_file_mocap="calib_sensor_marker_0724_static.yaml"
 calib_file_depth="calib_pose.yaml"
 #echo "setting_file=" ${setting_file}

 time=20190806102642

 ${PROJECT_SOURCE_DIR}/run/evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj.txt --plot ate.png --offset 0 --scale 1 --verbose
 ${PROJECT_SOURCE_DIR}/run/evaluate/evaluate_rpe.py ${seq_path}/groundtruth.txt traj.txt --fixed_delta  --delta 1 --delta_unit s --plot rpe.png --offset 0 --scale 1 --verbose
 
 # create the association file
 #${PROJECT_SOURCE_DIR}/run/evaluate/associate.py ${seq_path}/outputCalib.txt ${seq_path}/depth.txt --max_difference 0.05
 #${PROJECT_SOURCE_DIR}/run/evaluate/associate.py ${seq_path}/depth_hd.txt ${seq_path}/rgb_hd.txt
 #${PROJECT_SOURCE_DIR}/run/evaluate/associate.py association.txt ${seq_path}/groundtruth.txt 
 #${PROJECT_SOURCE_DIR}/run/evaluate/associate.py association.txt ${seq_path}/rgb.txt --max_difference 0.05
 #cp association.txt ${seq_path}/association.txt

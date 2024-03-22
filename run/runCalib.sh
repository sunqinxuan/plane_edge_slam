#!/bin/sh

 PROJECT_SOURCE_DIR="/home/sun/shadow_SLAM"
 setting_path=${PROJECT_SOURCE_DIR}"/run/settings/"
 setting_file=${PROJECT_SOURCE_DIR}"/run/settings/settings_kinect2.yaml"
 calibration_file=${PROJECT_SOURCE_DIR}"/run/settings/calib_sensor_marker.yaml"
 echo "setting_file=" ${setting_file}
 seq_path="/home/sun/dataset/data0804/scanning_3/"
 mode="chessLocalize"
 
 echo ${mode}

 if [ "${mode}" = "chessLocalize" ]
 then
 	echo "chess localization"
	${PROJECT_SOURCE_DIR}/run/evaluate/associate.py ${seq_path}/rgb_hd.txt ${seq_path}/groundtruth.txt
	cp association.txt ${seq_path}/association.txt
 elif [ "${mode}" = "CalibrateDynamic" ]
 then 
 	echo "calibrate dynamic"
	${PROJECT_SOURCE_DIR}/run/evaluate/associate.py ${seq_path}/outputCalib.txt ${seq_path}/outputGT.txt
	cp association.txt ${seq_path}/association.txt
 fi
 
 # main system
 bin/Kinect2QualisysCalibration ${seq_path} ${setting_path} ${mode}
 RETURN=$?
 echo "RETURN=" ${RETURN}

 if [ ${RETURN} -ne 0 ]
	then 
		echo "not equal"
		exit
 fi

 if [ "${mode}" = "chessLocalize" ]
 then
	cp outputCalib.txt ${seq_path}/outputCalib.txt
	cp outputGT.txt ${seq_path}/outputGT.txt
 fi

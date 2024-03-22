#!/bin/sh

seq_path="/media/sun/Elements/sun/rgbd_dataset_freiburg3_structure_notexture_near"
time="fr3_str_near"

run/evaluate_rpe.py ${seq_path}/groundtruth.txt ${time}/traj.txt ${time}/traj2.txt ${time}/traj3.txt --fixed_delta  --delta 1 --delta_unit s --plot ${time}/rpe_cmp_3.png --offset 0 --scale 1 --verbose

#evaluate/evaluate_rpe.py ${seq_path}/groundtruth.txt traj.txt --fixed_delta  --delta 1 --delta_unit s --plot ${time}/rpe.png --offset 0 --scale 1 --save ${time}/rpe.txt --verbose
#evaluate/evaluate_rpe.py ${seq_path}/groundtruth.txt traj2.txt --fixed_delta  --delta 1 --delta_unit s --plot ${time}/rpe2.png --offset 0 --scale 1 --save ${time}/rpe.txt --verbose

#evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj.txt --plot ${time}/ate.png --offset 0 --scale 1 --verbose
#evaluate/evaluate_ate.py ${seq_path}/groundtruth.txt traj2.txt --plot ${time}/ate2.png --offset 0 --scale 1 --verbose



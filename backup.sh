#!/bin/sh

 time=$(date "+%Y.%m.%d %H:%M:%S")

 git add .
 git commit -m "${time}"
 git push origin master

# echo "this is the backup for ${time}.\n\nfrom SqX" |mutt -s "backup" sunqinxuan@outlook.com -a /home/sun/shadow_SLAM/include/types.h /home/sun/shadow_SLAM/include/global_map.h /home/sun/shadow_SLAM/include/system_simulation.h /home/sun/shadow_SLAM/src/systems/system_simulation.cpp /home/sun/shadow_SLAM/src/Simulation.cpp /home/sun/shadow_SLAM/src/CMakeLists.txt /home/sun/shadow_SLAM/CMakeLists.txt

#/home/sun/shadow_SLAM/include/geometric_feature_matching.h /home/sun/shadow_SLAM/src/features/geometric_feature_matching.cpp /home/sun/shadow_SLAM/src/test.cpp /home/sun/Desktop/test/test.cpp /home/sun/shadow_SLAM/build/settings.yaml



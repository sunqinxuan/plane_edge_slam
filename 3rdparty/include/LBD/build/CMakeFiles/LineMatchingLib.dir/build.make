# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/sun/shadow_SLAM/3rdparty/include/LBD

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/sun/shadow_SLAM/3rdparty/include/LBD/build

# Include any dependencies generated for this target.
include CMakeFiles/LineMatchingLib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LineMatchingLib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LineMatchingLib.dir/flags.make

CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o: CMakeFiles/LineMatchingLib.dir/flags.make
CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o: ../PairwiseLineMatching.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sun/shadow_SLAM/3rdparty/include/LBD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o -c /home/sun/shadow_SLAM/3rdparty/include/LBD/PairwiseLineMatching.cpp

CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sun/shadow_SLAM/3rdparty/include/LBD/PairwiseLineMatching.cpp > CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.i

CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sun/shadow_SLAM/3rdparty/include/LBD/PairwiseLineMatching.cpp -o CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.s

CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.requires:

.PHONY : CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.requires

CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.provides: CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.requires
	$(MAKE) -f CMakeFiles/LineMatchingLib.dir/build.make CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.provides.build
.PHONY : CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.provides

CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.provides.build: CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o


CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o: CMakeFiles/LineMatchingLib.dir/flags.make
CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o: ../LineDescriptor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sun/shadow_SLAM/3rdparty/include/LBD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o -c /home/sun/shadow_SLAM/3rdparty/include/LBD/LineDescriptor.cpp

CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sun/shadow_SLAM/3rdparty/include/LBD/LineDescriptor.cpp > CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.i

CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sun/shadow_SLAM/3rdparty/include/LBD/LineDescriptor.cpp -o CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.s

CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.requires:

.PHONY : CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.requires

CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.provides: CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.requires
	$(MAKE) -f CMakeFiles/LineMatchingLib.dir/build.make CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.provides.build
.PHONY : CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.provides

CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.provides.build: CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o


CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o: CMakeFiles/LineMatchingLib.dir/flags.make
CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o: ../EDLineDetector.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/sun/shadow_SLAM/3rdparty/include/LBD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o -c /home/sun/shadow_SLAM/3rdparty/include/LBD/EDLineDetector.cpp

CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/sun/shadow_SLAM/3rdparty/include/LBD/EDLineDetector.cpp > CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.i

CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/sun/shadow_SLAM/3rdparty/include/LBD/EDLineDetector.cpp -o CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.s

CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.requires:

.PHONY : CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.requires

CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.provides: CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.requires
	$(MAKE) -f CMakeFiles/LineMatchingLib.dir/build.make CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.provides.build
.PHONY : CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.provides

CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.provides.build: CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o


# Object files for target LineMatchingLib
LineMatchingLib_OBJECTS = \
"CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o" \
"CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o" \
"CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o"

# External object files for target LineMatchingLib
LineMatchingLib_EXTERNAL_OBJECTS =

/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: CMakeFiles/LineMatchingLib.dir/build.make
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_ml.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_videostab.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_shape.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_viz.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_stitching.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_dnn.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_superres.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_objdetect.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_photo.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/lib/x86_64-linux-gnu/libsuperlu.so
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_video.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_calib3d.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_features2d.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_highgui.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_videoio.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_flann.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_imgcodecs.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_imgproc.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: /usr/local/lib/libopencv_core.so.3.4.6
/home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so: CMakeFiles/LineMatchingLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/sun/shadow_SLAM/3rdparty/include/LBD/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX shared library /home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LineMatchingLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LineMatchingLib.dir/build: /home/sun/shadow_SLAM/3rdparty/lib/libLineMatchingLib.so

.PHONY : CMakeFiles/LineMatchingLib.dir/build

CMakeFiles/LineMatchingLib.dir/requires: CMakeFiles/LineMatchingLib.dir/PairwiseLineMatching.cpp.o.requires
CMakeFiles/LineMatchingLib.dir/requires: CMakeFiles/LineMatchingLib.dir/LineDescriptor.cpp.o.requires
CMakeFiles/LineMatchingLib.dir/requires: CMakeFiles/LineMatchingLib.dir/EDLineDetector.cpp.o.requires

.PHONY : CMakeFiles/LineMatchingLib.dir/requires

CMakeFiles/LineMatchingLib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LineMatchingLib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LineMatchingLib.dir/clean

CMakeFiles/LineMatchingLib.dir/depend:
	cd /home/sun/shadow_SLAM/3rdparty/include/LBD/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/sun/shadow_SLAM/3rdparty/include/LBD /home/sun/shadow_SLAM/3rdparty/include/LBD /home/sun/shadow_SLAM/3rdparty/include/LBD/build /home/sun/shadow_SLAM/3rdparty/include/LBD/build /home/sun/shadow_SLAM/3rdparty/include/LBD/build/CMakeFiles/LineMatchingLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LineMatchingLib.dir/depend


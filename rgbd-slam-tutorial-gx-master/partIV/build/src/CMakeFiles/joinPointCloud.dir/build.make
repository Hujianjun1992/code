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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hxb/rgbd-slam-tutorial-gx-master/partIV

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build

# Include any dependencies generated for this target.
include src/CMakeFiles/joinPointCloud.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/joinPointCloud.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/joinPointCloud.dir/flags.make

src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o: src/CMakeFiles/joinPointCloud.dir/flags.make
src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o: ../src/joinPointCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o"
	cd /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o -c /home/hxb/rgbd-slam-tutorial-gx-master/partIV/src/joinPointCloud.cpp

src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.i"
	cd /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hxb/rgbd-slam-tutorial-gx-master/partIV/src/joinPointCloud.cpp > CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.i

src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.s"
	cd /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hxb/rgbd-slam-tutorial-gx-master/partIV/src/joinPointCloud.cpp -o CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.s

src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.requires:

.PHONY : src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.requires

src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.provides: src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/joinPointCloud.dir/build.make src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.provides.build
.PHONY : src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.provides

src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.provides.build: src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o


# Object files for target joinPointCloud
joinPointCloud_OBJECTS = \
"CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o"

# External object files for target joinPointCloud
joinPointCloud_EXTERNAL_OBJECTS =

../bin/joinPointCloud: src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o
../bin/joinPointCloud: src/CMakeFiles/joinPointCloud.dir/build.make
../bin/joinPointCloud: ../lib/libslambase.a
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_videostab.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ts.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_superres.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_stitching.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_contrib.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/joinPointCloud: /usr/lib/libpcl_common.so
../bin/joinPointCloud: /usr/lib/libpcl_octree.so
../bin/joinPointCloud: /usr/lib/libOpenNI.so
../bin/joinPointCloud: /usr/lib/libpcl_io.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/joinPointCloud: /usr/lib/libpcl_kdtree.so
../bin/joinPointCloud: /usr/lib/libpcl_search.so
../bin/joinPointCloud: /usr/lib/libpcl_visualization.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/joinPointCloud: /usr/lib/libOpenNI.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/joinPointCloud: /usr/lib/libvtkCharts.so.5.8.0
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_nonfree.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ocl.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_gpu.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_photo.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_objdetect.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_legacy.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_video.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_ml.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_features2d.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_highgui.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_imgproc.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_flann.so.2.4.8
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libopencv_core.so.2.4.8
../bin/joinPointCloud: /usr/lib/libvtkViews.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkInfovis.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkWidgets.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkHybrid.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkParallel.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkRendering.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkGraphics.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkImaging.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkIO.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkFiltering.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtkCommon.so.5.8.0
../bin/joinPointCloud: /usr/lib/libvtksys.so.5.8.0
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/joinPointCloud: /usr/lib/libpcl_common.so
../bin/joinPointCloud: /usr/lib/libpcl_octree.so
../bin/joinPointCloud: /usr/lib/libOpenNI.so
../bin/joinPointCloud: /usr/lib/libpcl_io.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/joinPointCloud: /usr/lib/libpcl_kdtree.so
../bin/joinPointCloud: /usr/lib/libpcl_search.so
../bin/joinPointCloud: /usr/lib/libpcl_visualization.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/joinPointCloud: /usr/lib/libpcl_common.so
../bin/joinPointCloud: /usr/lib/libpcl_octree.so
../bin/joinPointCloud: /usr/lib/libOpenNI.so
../bin/joinPointCloud: /usr/lib/libpcl_io.so
../bin/joinPointCloud: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/joinPointCloud: /usr/lib/libpcl_kdtree.so
../bin/joinPointCloud: /usr/lib/libpcl_search.so
../bin/joinPointCloud: /usr/lib/libpcl_visualization.so
../bin/joinPointCloud: src/CMakeFiles/joinPointCloud.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/joinPointCloud"
	cd /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/joinPointCloud.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/joinPointCloud.dir/build: ../bin/joinPointCloud

.PHONY : src/CMakeFiles/joinPointCloud.dir/build

src/CMakeFiles/joinPointCloud.dir/requires: src/CMakeFiles/joinPointCloud.dir/joinPointCloud.cpp.o.requires

.PHONY : src/CMakeFiles/joinPointCloud.dir/requires

src/CMakeFiles/joinPointCloud.dir/clean:
	cd /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src && $(CMAKE_COMMAND) -P CMakeFiles/joinPointCloud.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/joinPointCloud.dir/clean

src/CMakeFiles/joinPointCloud.dir/depend:
	cd /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hxb/rgbd-slam-tutorial-gx-master/partIV /home/hxb/rgbd-slam-tutorial-gx-master/partIV/src /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src /home/hxb/rgbd-slam-tutorial-gx-master/partIV/build/src/CMakeFiles/joinPointCloud.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/joinPointCloud.dir/depend


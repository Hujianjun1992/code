# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.6

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
CMAKE_SOURCE_DIR = /home/hxb/rgbdslam/6

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hxb/rgbdslam/6/build

# Include any dependencies generated for this target.
include src/CMakeFiles/slamEnd.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/slamEnd.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/slamEnd.dir/flags.make

src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o: src/CMakeFiles/slamEnd.dir/flags.make
src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o: ../src/slamEnd.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hxb/rgbdslam/6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o"
	cd /home/hxb/rgbdslam/6/build/src && g++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/slamEnd.dir/slamEnd.cpp.o -c /home/hxb/rgbdslam/6/src/slamEnd.cpp

src/CMakeFiles/slamEnd.dir/slamEnd.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/slamEnd.dir/slamEnd.cpp.i"
	cd /home/hxb/rgbdslam/6/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hxb/rgbdslam/6/src/slamEnd.cpp > CMakeFiles/slamEnd.dir/slamEnd.cpp.i

src/CMakeFiles/slamEnd.dir/slamEnd.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/slamEnd.dir/slamEnd.cpp.s"
	cd /home/hxb/rgbdslam/6/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hxb/rgbdslam/6/src/slamEnd.cpp -o CMakeFiles/slamEnd.dir/slamEnd.cpp.s

src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.requires:

.PHONY : src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.requires

src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.provides: src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/slamEnd.dir/build.make src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.provides.build
.PHONY : src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.provides

src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.provides.build: src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o


# Object files for target slamEnd
slamEnd_OBJECTS = \
"CMakeFiles/slamEnd.dir/slamEnd.cpp.o"

# External object files for target slamEnd
slamEnd_EXTERNAL_OBJECTS =

../bin/slamEnd: src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o
../bin/slamEnd: src/CMakeFiles/slamEnd.dir/build.make
../bin/slamEnd: ../lib/libslambase.a
../bin/slamEnd: /usr/local/lib/libopencv_videostab.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_ts.a
../bin/slamEnd: /usr/local/lib/libopencv_superres.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_stitching.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_contrib.so.2.4.13
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/slamEnd: /usr/lib/libpcl_common.so
../bin/slamEnd: /usr/lib/libpcl_octree.so
../bin/slamEnd: /usr/lib/libOpenNI.so
../bin/slamEnd: /usr/lib/libpcl_io.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/slamEnd: /usr/lib/libpcl_kdtree.so
../bin/slamEnd: /usr/lib/libpcl_search.so
../bin/slamEnd: /usr/lib/libpcl_visualization.so
../bin/slamEnd: /usr/lib/libpcl_sample_consensus.so
../bin/slamEnd: /usr/lib/libpcl_filters.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/slamEnd: /usr/lib/libOpenNI.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/slamEnd: /usr/lib/libvtkCharts.so.5.8.0
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libcxsparse.so
../bin/slamEnd: /usr/local/lib/libopencv_nonfree.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_ocl.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_gpu.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_photo.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_objdetect.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_legacy.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_video.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_ml.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_calib3d.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_features2d.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_highgui.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_imgproc.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_flann.so.2.4.13
../bin/slamEnd: /usr/local/lib/libopencv_core.so.2.4.13
../bin/slamEnd: /usr/lib/libvtkViews.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkInfovis.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkWidgets.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkHybrid.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkParallel.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkVolumeRendering.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkRendering.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkGraphics.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkImaging.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkIO.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkFiltering.so.5.8.0
../bin/slamEnd: /usr/lib/libvtkCommon.so.5.8.0
../bin/slamEnd: /usr/lib/libvtksys.so.5.8.0
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/slamEnd: /usr/lib/libpcl_common.so
../bin/slamEnd: /usr/lib/libpcl_octree.so
../bin/slamEnd: /usr/lib/libOpenNI.so
../bin/slamEnd: /usr/lib/libpcl_io.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/slamEnd: /usr/lib/libpcl_kdtree.so
../bin/slamEnd: /usr/lib/libpcl_search.so
../bin/slamEnd: /usr/lib/libpcl_visualization.so
../bin/slamEnd: /usr/lib/libpcl_sample_consensus.so
../bin/slamEnd: /usr/lib/libpcl_filters.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_system.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../bin/slamEnd: /usr/lib/libpcl_common.so
../bin/slamEnd: /usr/lib/libpcl_octree.so
../bin/slamEnd: /usr/lib/libOpenNI.so
../bin/slamEnd: /usr/lib/libpcl_io.so
../bin/slamEnd: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../bin/slamEnd: /usr/lib/libpcl_kdtree.so
../bin/slamEnd: /usr/lib/libpcl_search.so
../bin/slamEnd: /usr/lib/libpcl_visualization.so
../bin/slamEnd: /usr/lib/libpcl_sample_consensus.so
../bin/slamEnd: /usr/lib/libpcl_filters.so
../bin/slamEnd: src/CMakeFiles/slamEnd.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hxb/rgbdslam/6/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/slamEnd"
	cd /home/hxb/rgbdslam/6/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/slamEnd.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/slamEnd.dir/build: ../bin/slamEnd

.PHONY : src/CMakeFiles/slamEnd.dir/build

src/CMakeFiles/slamEnd.dir/requires: src/CMakeFiles/slamEnd.dir/slamEnd.cpp.o.requires

.PHONY : src/CMakeFiles/slamEnd.dir/requires

src/CMakeFiles/slamEnd.dir/clean:
	cd /home/hxb/rgbdslam/6/build/src && $(CMAKE_COMMAND) -P CMakeFiles/slamEnd.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/slamEnd.dir/clean

src/CMakeFiles/slamEnd.dir/depend:
	cd /home/hxb/rgbdslam/6/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hxb/rgbdslam/6 /home/hxb/rgbdslam/6/src /home/hxb/rgbdslam/6/build /home/hxb/rgbdslam/6/build/src /home/hxb/rgbdslam/6/build/src/CMakeFiles/slamEnd.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/slamEnd.dir/depend

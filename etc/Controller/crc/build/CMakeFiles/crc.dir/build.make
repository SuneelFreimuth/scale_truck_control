# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build

# Include any dependencies generated for this target.
include CMakeFiles/crc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/crc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/crc.dir/flags.make

CMakeFiles/crc.dir/main.cpp.o: CMakeFiles/crc.dir/flags.make
CMakeFiles/crc.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/crc.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crc.dir/main.cpp.o -c /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/main.cpp

CMakeFiles/crc.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crc.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/main.cpp > CMakeFiles/crc.dir/main.cpp.i

CMakeFiles/crc.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crc.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/main.cpp -o CMakeFiles/crc.dir/main.cpp.s

CMakeFiles/crc.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/crc.dir/main.cpp.o.requires

CMakeFiles/crc.dir/main.cpp.o.provides: CMakeFiles/crc.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/crc.dir/build.make CMakeFiles/crc.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/crc.dir/main.cpp.o.provides

CMakeFiles/crc.dir/main.cpp.o.provides.build: CMakeFiles/crc.dir/main.cpp.o


CMakeFiles/crc.dir/crc.cpp.o: CMakeFiles/crc.dir/flags.make
CMakeFiles/crc.dir/crc.cpp.o: ../crc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/crc.dir/crc.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crc.dir/crc.cpp.o -c /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/crc.cpp

CMakeFiles/crc.dir/crc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crc.dir/crc.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/crc.cpp > CMakeFiles/crc.dir/crc.cpp.i

CMakeFiles/crc.dir/crc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crc.dir/crc.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/crc.cpp -o CMakeFiles/crc.dir/crc.cpp.s

CMakeFiles/crc.dir/crc.cpp.o.requires:

.PHONY : CMakeFiles/crc.dir/crc.cpp.o.requires

CMakeFiles/crc.dir/crc.cpp.o.provides: CMakeFiles/crc.dir/crc.cpp.o.requires
	$(MAKE) -f CMakeFiles/crc.dir/build.make CMakeFiles/crc.dir/crc.cpp.o.provides.build
.PHONY : CMakeFiles/crc.dir/crc.cpp.o.provides

CMakeFiles/crc.dir/crc.cpp.o.provides.build: CMakeFiles/crc.dir/crc.cpp.o


CMakeFiles/crc.dir/sock_udp.cpp.o: CMakeFiles/crc.dir/flags.make
CMakeFiles/crc.dir/sock_udp.cpp.o: ../sock_udp.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/crc.dir/sock_udp.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/crc.dir/sock_udp.cpp.o -c /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/sock_udp.cpp

CMakeFiles/crc.dir/sock_udp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/crc.dir/sock_udp.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/sock_udp.cpp > CMakeFiles/crc.dir/sock_udp.cpp.i

CMakeFiles/crc.dir/sock_udp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/crc.dir/sock_udp.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/sock_udp.cpp -o CMakeFiles/crc.dir/sock_udp.cpp.s

CMakeFiles/crc.dir/sock_udp.cpp.o.requires:

.PHONY : CMakeFiles/crc.dir/sock_udp.cpp.o.requires

CMakeFiles/crc.dir/sock_udp.cpp.o.provides: CMakeFiles/crc.dir/sock_udp.cpp.o.requires
	$(MAKE) -f CMakeFiles/crc.dir/build.make CMakeFiles/crc.dir/sock_udp.cpp.o.provides.build
.PHONY : CMakeFiles/crc.dir/sock_udp.cpp.o.provides

CMakeFiles/crc.dir/sock_udp.cpp.o.provides.build: CMakeFiles/crc.dir/sock_udp.cpp.o


# Object files for target crc
crc_OBJECTS = \
"CMakeFiles/crc.dir/main.cpp.o" \
"CMakeFiles/crc.dir/crc.cpp.o" \
"CMakeFiles/crc.dir/sock_udp.cpp.o"

# External object files for target crc
crc_EXTERNAL_OBJECTS =

crc: CMakeFiles/crc.dir/main.cpp.o
crc: CMakeFiles/crc.dir/crc.cpp.o
crc: CMakeFiles/crc.dir/sock_udp.cpp.o
crc: CMakeFiles/crc.dir/build.make
crc: /usr/local/lib/libopencv_gapi.so.4.4.0
crc: /usr/local/lib/libopencv_stitching.so.4.4.0
crc: /usr/local/lib/libopencv_alphamat.so.4.4.0
crc: /usr/local/lib/libopencv_aruco.so.4.4.0
crc: /usr/local/lib/libopencv_bgsegm.so.4.4.0
crc: /usr/local/lib/libopencv_bioinspired.so.4.4.0
crc: /usr/local/lib/libopencv_ccalib.so.4.4.0
crc: /usr/local/lib/libopencv_cudabgsegm.so.4.4.0
crc: /usr/local/lib/libopencv_cudafeatures2d.so.4.4.0
crc: /usr/local/lib/libopencv_cudaobjdetect.so.4.4.0
crc: /usr/local/lib/libopencv_cudastereo.so.4.4.0
crc: /usr/local/lib/libopencv_dnn_objdetect.so.4.4.0
crc: /usr/local/lib/libopencv_dnn_superres.so.4.4.0
crc: /usr/local/lib/libopencv_dpm.so.4.4.0
crc: /usr/local/lib/libopencv_face.so.4.4.0
crc: /usr/local/lib/libopencv_freetype.so.4.4.0
crc: /usr/local/lib/libopencv_fuzzy.so.4.4.0
crc: /usr/local/lib/libopencv_hdf.so.4.4.0
crc: /usr/local/lib/libopencv_hfs.so.4.4.0
crc: /usr/local/lib/libopencv_img_hash.so.4.4.0
crc: /usr/local/lib/libopencv_intensity_transform.so.4.4.0
crc: /usr/local/lib/libopencv_line_descriptor.so.4.4.0
crc: /usr/local/lib/libopencv_quality.so.4.4.0
crc: /usr/local/lib/libopencv_rapid.so.4.4.0
crc: /usr/local/lib/libopencv_reg.so.4.4.0
crc: /usr/local/lib/libopencv_rgbd.so.4.4.0
crc: /usr/local/lib/libopencv_saliency.so.4.4.0
crc: /usr/local/lib/libopencv_stereo.so.4.4.0
crc: /usr/local/lib/libopencv_structured_light.so.4.4.0
crc: /usr/local/lib/libopencv_superres.so.4.4.0
crc: /usr/local/lib/libopencv_surface_matching.so.4.4.0
crc: /usr/local/lib/libopencv_tracking.so.4.4.0
crc: /usr/local/lib/libopencv_videostab.so.4.4.0
crc: /usr/local/lib/libopencv_xfeatures2d.so.4.4.0
crc: /usr/local/lib/libopencv_xobjdetect.so.4.4.0
crc: /usr/local/lib/libopencv_xphoto.so.4.4.0
crc: /usr/local/lib/libopencv_shape.so.4.4.0
crc: /usr/local/lib/libopencv_highgui.so.4.4.0
crc: /usr/local/lib/libopencv_datasets.so.4.4.0
crc: /usr/local/lib/libopencv_plot.so.4.4.0
crc: /usr/local/lib/libopencv_text.so.4.4.0
crc: /usr/local/lib/libopencv_dnn.so.4.4.0
crc: /usr/local/lib/libopencv_ml.so.4.4.0
crc: /usr/local/lib/libopencv_phase_unwrapping.so.4.4.0
crc: /usr/local/lib/libopencv_cudacodec.so.4.4.0
crc: /usr/local/lib/libopencv_videoio.so.4.4.0
crc: /usr/local/lib/libopencv_cudaoptflow.so.4.4.0
crc: /usr/local/lib/libopencv_cudalegacy.so.4.4.0
crc: /usr/local/lib/libopencv_cudawarping.so.4.4.0
crc: /usr/local/lib/libopencv_optflow.so.4.4.0
crc: /usr/local/lib/libopencv_ximgproc.so.4.4.0
crc: /usr/local/lib/libopencv_video.so.4.4.0
crc: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
crc: /usr/local/lib/libopencv_objdetect.so.4.4.0
crc: /usr/local/lib/libopencv_calib3d.so.4.4.0
crc: /usr/local/lib/libopencv_features2d.so.4.4.0
crc: /usr/local/lib/libopencv_flann.so.4.4.0
crc: /usr/local/lib/libopencv_photo.so.4.4.0
crc: /usr/local/lib/libopencv_cudaimgproc.so.4.4.0
crc: /usr/local/lib/libopencv_cudafilters.so.4.4.0
crc: /usr/local/lib/libopencv_imgproc.so.4.4.0
crc: /usr/local/lib/libopencv_cudaarithm.so.4.4.0
crc: /usr/local/lib/libopencv_core.so.4.4.0
crc: /usr/local/lib/libopencv_cudev.so.4.4.0
crc: CMakeFiles/crc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable crc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/crc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/crc.dir/build: crc

.PHONY : CMakeFiles/crc.dir/build

CMakeFiles/crc.dir/requires: CMakeFiles/crc.dir/main.cpp.o.requires
CMakeFiles/crc.dir/requires: CMakeFiles/crc.dir/crc.cpp.o.requires
CMakeFiles/crc.dir/requires: CMakeFiles/crc.dir/sock_udp.cpp.o.requires

.PHONY : CMakeFiles/crc.dir/requires

CMakeFiles/crc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/crc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/crc.dir/clean

CMakeFiles/crc.dir/depend:
	cd /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build /home/avees/catkin_ws/src/scale_truck_control/etc/Controller/crc/build/CMakeFiles/crc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/crc.dir/depend


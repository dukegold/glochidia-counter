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
CMAKE_COMMAND = /usr/bin/cmake.exe

# The command to remove a file.
RM = /usr/bin/cmake.exe -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/LiuZe/glochidia-counter

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/LiuZe/glochidia-counter/build

# Include any dependencies generated for this target.
include CMakeFiles/glochidia_counter_bin.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/glochidia_counter_bin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/glochidia_counter_bin.dir/flags.make

CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o: CMakeFiles/glochidia_counter_bin.dir/flags.make
CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o: ../Demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/LiuZe/glochidia-counter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o"
	/usr/bin/c++.exe   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o -c /home/LiuZe/glochidia-counter/Demo.cpp

CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.i"
	/usr/bin/c++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/LiuZe/glochidia-counter/Demo.cpp > CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.i

CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.s"
	/usr/bin/c++.exe  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/LiuZe/glochidia-counter/Demo.cpp -o CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.s

CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.requires:

.PHONY : CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.requires

CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.provides: CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.requires
	$(MAKE) -f CMakeFiles/glochidia_counter_bin.dir/build.make CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.provides.build
.PHONY : CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.provides

CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.provides.build: CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o


# Object files for target glochidia_counter_bin
glochidia_counter_bin_OBJECTS = \
"CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o"

# External object files for target glochidia_counter_bin
glochidia_counter_bin_EXTERNAL_OBJECTS =

glochidia_counter.exe: CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o
glochidia_counter.exe: CMakeFiles/glochidia_counter_bin.dir/build.make
glochidia_counter.exe: libglochidia_counter.a
glochidia_counter.exe: /usr/local/lib/libopencv_xphoto.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_xobjdetect.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_tracking.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_surface_matching.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_structured_light.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_stereo.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_sfm.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_saliency.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_rgbd.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_reg.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_plot.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_optflow.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_ximgproc.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_line_descriptor.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_hdf.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_fuzzy.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_dpm.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_dnn.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_datasets.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_text.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_face.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_ccalib.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_bioinspired.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_bgsegm.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_aruco.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_viz.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_videostab.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_superres.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_stitching.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_xfeatures2d.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_shape.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_video.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_photo.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_objdetect.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_calib3d.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_features2d.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_ml.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_highgui.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_videoio.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_imgcodecs.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_imgproc.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_flann.dll.a
glochidia_counter.exe: /usr/local/lib/libopencv_core.dll.a
glochidia_counter.exe: CMakeFiles/glochidia_counter_bin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/LiuZe/glochidia-counter/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable glochidia_counter.exe"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/glochidia_counter_bin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/glochidia_counter_bin.dir/build: glochidia_counter.exe

.PHONY : CMakeFiles/glochidia_counter_bin.dir/build

CMakeFiles/glochidia_counter_bin.dir/requires: CMakeFiles/glochidia_counter_bin.dir/Demo.cpp.o.requires

.PHONY : CMakeFiles/glochidia_counter_bin.dir/requires

CMakeFiles/glochidia_counter_bin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/glochidia_counter_bin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/glochidia_counter_bin.dir/clean

CMakeFiles/glochidia_counter_bin.dir/depend:
	cd /home/LiuZe/glochidia-counter/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/LiuZe/glochidia-counter /home/LiuZe/glochidia-counter /home/LiuZe/glochidia-counter/build /home/LiuZe/glochidia-counter/build /home/LiuZe/glochidia-counter/build/CMakeFiles/glochidia_counter_bin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/glochidia_counter_bin.dir/depend

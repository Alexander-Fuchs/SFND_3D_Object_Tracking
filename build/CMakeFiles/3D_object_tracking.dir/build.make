# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /Applications/CMake.app/Contents/bin/cmake

# The command to remove a file.
RM = /Applications/CMake.app/Contents/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build

# Include any dependencies generated for this target.
include CMakeFiles/3D_object_tracking.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/3D_object_tracking.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/3D_object_tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/3D_object_tracking.dir/flags.make

CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o: CMakeFiles/3D_object_tracking.dir/flags.make
CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o: /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/camFusion_Student.cpp
CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o: CMakeFiles/3D_object_tracking.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o -MF CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o.d -o CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o -c /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/camFusion_Student.cpp

CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/camFusion_Student.cpp > CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.i

CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/camFusion_Student.cpp -o CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.s

CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o: CMakeFiles/3D_object_tracking.dir/flags.make
CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o: /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/FinalProject_Camera.cpp
CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o: CMakeFiles/3D_object_tracking.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o -MF CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o.d -o CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o -c /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/FinalProject_Camera.cpp

CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/FinalProject_Camera.cpp > CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.i

CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/FinalProject_Camera.cpp -o CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.s

CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o: CMakeFiles/3D_object_tracking.dir/flags.make
CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o: /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/lidarData.cpp
CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o: CMakeFiles/3D_object_tracking.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o -MF CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o.d -o CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o -c /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/lidarData.cpp

CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/lidarData.cpp > CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.i

CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/lidarData.cpp -o CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.s

CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o: CMakeFiles/3D_object_tracking.dir/flags.make
CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o: /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/matching2D_Student.cpp
CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o: CMakeFiles/3D_object_tracking.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o -MF CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o.d -o CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o -c /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/matching2D_Student.cpp

CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/matching2D_Student.cpp > CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.i

CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/matching2D_Student.cpp -o CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.s

CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o: CMakeFiles/3D_object_tracking.dir/flags.make
CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o: /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/objectDetection2D.cpp
CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o: CMakeFiles/3D_object_tracking.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o -MF CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o.d -o CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o -c /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/objectDetection2D.cpp

CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/objectDetection2D.cpp > CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.i

CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/src/objectDetection2D.cpp -o CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.s

# Object files for target 3D_object_tracking
3D_object_tracking_OBJECTS = \
"CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o" \
"CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o" \
"CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o" \
"CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o" \
"CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o"

# External object files for target 3D_object_tracking
3D_object_tracking_EXTERNAL_OBJECTS =

3D_object_tracking: CMakeFiles/3D_object_tracking.dir/src/camFusion_Student.cpp.o
3D_object_tracking: CMakeFiles/3D_object_tracking.dir/src/FinalProject_Camera.cpp.o
3D_object_tracking: CMakeFiles/3D_object_tracking.dir/src/lidarData.cpp.o
3D_object_tracking: CMakeFiles/3D_object_tracking.dir/src/matching2D_Student.cpp.o
3D_object_tracking: CMakeFiles/3D_object_tracking.dir/src/objectDetection2D.cpp.o
3D_object_tracking: CMakeFiles/3D_object_tracking.dir/build.make
3D_object_tracking: /opt/homebrew/lib/libopencv_gapi.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_stitching.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_alphamat.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_aruco.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_barcode.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_bgsegm.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_bioinspired.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_ccalib.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_dnn_objdetect.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_dnn_superres.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_dpm.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_face.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_freetype.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_fuzzy.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_hfs.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_img_hash.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_intensity_transform.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_line_descriptor.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_mcc.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_quality.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_rapid.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_reg.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_rgbd.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_saliency.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_sfm.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_stereo.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_structured_light.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_superres.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_surface_matching.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_tracking.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_videostab.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_viz.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_wechat_qrcode.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_xfeatures2d.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_xobjdetect.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_xphoto.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_shape.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_highgui.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_datasets.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_plot.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_text.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_ml.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_phase_unwrapping.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_optflow.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_ximgproc.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_video.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_videoio.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_imgcodecs.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_objdetect.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_calib3d.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_dnn.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_features2d.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_flann.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_photo.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_imgproc.4.7.0.dylib
3D_object_tracking: /opt/homebrew/lib/libopencv_core.4.7.0.dylib
3D_object_tracking: CMakeFiles/3D_object_tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX executable 3D_object_tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/3D_object_tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/3D_object_tracking.dir/build: 3D_object_tracking
.PHONY : CMakeFiles/3D_object_tracking.dir/build

CMakeFiles/3D_object_tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/3D_object_tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/3D_object_tracking.dir/clean

CMakeFiles/3D_object_tracking.dir/depend:
	cd /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build /Users/alexanderfuchs/Desktop/SFND_3D_Object_Tracking-master/build/CMakeFiles/3D_object_tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/3D_object_tracking.dir/depend


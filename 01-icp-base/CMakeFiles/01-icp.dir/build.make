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
CMAKE_SOURCE_DIR = /home/misham/MIRI/GPR/01-icp-base

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/misham/MIRI/GPR/01-icp-base

# Include any dependencies generated for this target.
include CMakeFiles/01-icp.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/01-icp.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/01-icp.dir/flags.make

CMakeFiles/01-icp.dir/imgui/imgui.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/imgui/imgui.cpp.o: imgui/imgui.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/01-icp.dir/imgui/imgui.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/imgui/imgui.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/imgui/imgui.cpp

CMakeFiles/01-icp.dir/imgui/imgui.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/imgui/imgui.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/imgui/imgui.cpp > CMakeFiles/01-icp.dir/imgui/imgui.cpp.i

CMakeFiles/01-icp.dir/imgui/imgui.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/imgui/imgui.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/imgui/imgui.cpp -o CMakeFiles/01-icp.dir/imgui/imgui.cpp.s

CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.requires

CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.provides: CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.provides

CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.provides.build: CMakeFiles/01-icp.dir/imgui/imgui.cpp.o


CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o: imgui/imgui_demo.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_demo.cpp

CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_demo.cpp > CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.i

CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_demo.cpp -o CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.s

CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.requires

CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.provides: CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.provides

CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.provides.build: CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o


CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o: imgui/imgui_draw.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_draw.cpp

CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_draw.cpp > CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.i

CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_draw.cpp -o CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.s

CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.requires

CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.provides: CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.provides

CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.provides.build: CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o


CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o: imgui/imgui_widgets.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_widgets.cpp

CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_widgets.cpp > CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.i

CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_widgets.cpp -o CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.s

CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.requires

CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.provides: CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.provides

CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.provides.build: CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o


CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o: imgui/imgui_impl_opengl3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_impl_opengl3.cpp

CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_impl_opengl3.cpp > CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.i

CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_impl_opengl3.cpp -o CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.s

CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.requires

CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.provides: CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.provides

CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.provides.build: CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o


CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o: imgui/imgui_impl_glut.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_impl_glut.cpp

CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_impl_glut.cpp > CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.i

CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/imgui/imgui_impl_glut.cpp -o CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.s

CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.requires

CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.provides: CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.provides

CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.provides.build: CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o


CMakeFiles/01-icp.dir/gl3w/gl3w.c.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/gl3w/gl3w.c.o: gl3w/gl3w.c
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object CMakeFiles/01-icp.dir/gl3w/gl3w.c.o"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -o CMakeFiles/01-icp.dir/gl3w/gl3w.c.o   -c /home/misham/MIRI/GPR/01-icp-base/gl3w/gl3w.c

CMakeFiles/01-icp.dir/gl3w/gl3w.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/01-icp.dir/gl3w/gl3w.c.i"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/gl3w/gl3w.c > CMakeFiles/01-icp.dir/gl3w/gl3w.c.i

CMakeFiles/01-icp.dir/gl3w/gl3w.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/01-icp.dir/gl3w/gl3w.c.s"
	/usr/bin/cc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/gl3w/gl3w.c -o CMakeFiles/01-icp.dir/gl3w/gl3w.c.s

CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.requires:

.PHONY : CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.requires

CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.provides: CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.provides

CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.provides.build: CMakeFiles/01-icp.dir/gl3w/gl3w.c.o


CMakeFiles/01-icp.dir/PointCloud.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/PointCloud.cpp.o: PointCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/01-icp.dir/PointCloud.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/PointCloud.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/PointCloud.cpp

CMakeFiles/01-icp.dir/PointCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/PointCloud.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/PointCloud.cpp > CMakeFiles/01-icp.dir/PointCloud.cpp.i

CMakeFiles/01-icp.dir/PointCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/PointCloud.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/PointCloud.cpp -o CMakeFiles/01-icp.dir/PointCloud.cpp.s

CMakeFiles/01-icp.dir/PointCloud.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/PointCloud.cpp.o.requires

CMakeFiles/01-icp.dir/PointCloud.cpp.o.provides: CMakeFiles/01-icp.dir/PointCloud.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/PointCloud.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/PointCloud.cpp.o.provides

CMakeFiles/01-icp.dir/PointCloud.cpp.o.provides.build: CMakeFiles/01-icp.dir/PointCloud.cpp.o


CMakeFiles/01-icp.dir/Camera.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/Camera.cpp.o: Camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/01-icp.dir/Camera.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/Camera.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/Camera.cpp

CMakeFiles/01-icp.dir/Camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/Camera.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/Camera.cpp > CMakeFiles/01-icp.dir/Camera.cpp.i

CMakeFiles/01-icp.dir/Camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/Camera.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/Camera.cpp -o CMakeFiles/01-icp.dir/Camera.cpp.s

CMakeFiles/01-icp.dir/Camera.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/Camera.cpp.o.requires

CMakeFiles/01-icp.dir/Camera.cpp.o.provides: CMakeFiles/01-icp.dir/Camera.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/Camera.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/Camera.cpp.o.provides

CMakeFiles/01-icp.dir/Camera.cpp.o.provides.build: CMakeFiles/01-icp.dir/Camera.cpp.o


CMakeFiles/01-icp.dir/Scene.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/Scene.cpp.o: Scene.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Building CXX object CMakeFiles/01-icp.dir/Scene.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/Scene.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/Scene.cpp

CMakeFiles/01-icp.dir/Scene.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/Scene.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/Scene.cpp > CMakeFiles/01-icp.dir/Scene.cpp.i

CMakeFiles/01-icp.dir/Scene.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/Scene.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/Scene.cpp -o CMakeFiles/01-icp.dir/Scene.cpp.s

CMakeFiles/01-icp.dir/Scene.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/Scene.cpp.o.requires

CMakeFiles/01-icp.dir/Scene.cpp.o.provides: CMakeFiles/01-icp.dir/Scene.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/Scene.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/Scene.cpp.o.provides

CMakeFiles/01-icp.dir/Scene.cpp.o.provides.build: CMakeFiles/01-icp.dir/Scene.cpp.o


CMakeFiles/01-icp.dir/Shader.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/Shader.cpp.o: Shader.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_11) "Building CXX object CMakeFiles/01-icp.dir/Shader.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/Shader.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/Shader.cpp

CMakeFiles/01-icp.dir/Shader.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/Shader.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/Shader.cpp > CMakeFiles/01-icp.dir/Shader.cpp.i

CMakeFiles/01-icp.dir/Shader.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/Shader.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/Shader.cpp -o CMakeFiles/01-icp.dir/Shader.cpp.s

CMakeFiles/01-icp.dir/Shader.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/Shader.cpp.o.requires

CMakeFiles/01-icp.dir/Shader.cpp.o.provides: CMakeFiles/01-icp.dir/Shader.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/Shader.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/Shader.cpp.o.provides

CMakeFiles/01-icp.dir/Shader.cpp.o.provides.build: CMakeFiles/01-icp.dir/Shader.cpp.o


CMakeFiles/01-icp.dir/ShaderProgram.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/ShaderProgram.cpp.o: ShaderProgram.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_12) "Building CXX object CMakeFiles/01-icp.dir/ShaderProgram.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/ShaderProgram.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/ShaderProgram.cpp

CMakeFiles/01-icp.dir/ShaderProgram.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/ShaderProgram.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/ShaderProgram.cpp > CMakeFiles/01-icp.dir/ShaderProgram.cpp.i

CMakeFiles/01-icp.dir/ShaderProgram.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/ShaderProgram.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/ShaderProgram.cpp -o CMakeFiles/01-icp.dir/ShaderProgram.cpp.s

CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.requires

CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.provides: CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.provides

CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.provides.build: CMakeFiles/01-icp.dir/ShaderProgram.cpp.o


CMakeFiles/01-icp.dir/Application.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/Application.cpp.o: Application.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_13) "Building CXX object CMakeFiles/01-icp.dir/Application.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/Application.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/Application.cpp

CMakeFiles/01-icp.dir/Application.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/Application.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/Application.cpp > CMakeFiles/01-icp.dir/Application.cpp.i

CMakeFiles/01-icp.dir/Application.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/Application.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/Application.cpp -o CMakeFiles/01-icp.dir/Application.cpp.s

CMakeFiles/01-icp.dir/Application.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/Application.cpp.o.requires

CMakeFiles/01-icp.dir/Application.cpp.o.provides: CMakeFiles/01-icp.dir/Application.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/Application.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/Application.cpp.o.provides

CMakeFiles/01-icp.dir/Application.cpp.o.provides.build: CMakeFiles/01-icp.dir/Application.cpp.o


CMakeFiles/01-icp.dir/main.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_14) "Building CXX object CMakeFiles/01-icp.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/main.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/main.cpp

CMakeFiles/01-icp.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/main.cpp > CMakeFiles/01-icp.dir/main.cpp.i

CMakeFiles/01-icp.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/main.cpp -o CMakeFiles/01-icp.dir/main.cpp.s

CMakeFiles/01-icp.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/main.cpp.o.requires

CMakeFiles/01-icp.dir/main.cpp.o.provides: CMakeFiles/01-icp.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/main.cpp.o.provides

CMakeFiles/01-icp.dir/main.cpp.o.provides.build: CMakeFiles/01-icp.dir/main.cpp.o


CMakeFiles/01-icp.dir/NormalEstimator.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/NormalEstimator.cpp.o: NormalEstimator.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_15) "Building CXX object CMakeFiles/01-icp.dir/NormalEstimator.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/NormalEstimator.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/NormalEstimator.cpp

CMakeFiles/01-icp.dir/NormalEstimator.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/NormalEstimator.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/NormalEstimator.cpp > CMakeFiles/01-icp.dir/NormalEstimator.cpp.i

CMakeFiles/01-icp.dir/NormalEstimator.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/NormalEstimator.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/NormalEstimator.cpp -o CMakeFiles/01-icp.dir/NormalEstimator.cpp.s

CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.requires

CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.provides: CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.provides

CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.provides.build: CMakeFiles/01-icp.dir/NormalEstimator.cpp.o


CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o: NearestNeighbors.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_16) "Building CXX object CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/NearestNeighbors.cpp

CMakeFiles/01-icp.dir/NearestNeighbors.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/NearestNeighbors.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/NearestNeighbors.cpp > CMakeFiles/01-icp.dir/NearestNeighbors.cpp.i

CMakeFiles/01-icp.dir/NearestNeighbors.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/NearestNeighbors.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/NearestNeighbors.cpp -o CMakeFiles/01-icp.dir/NearestNeighbors.cpp.s

CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.requires

CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.provides: CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.provides

CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.provides.build: CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o


CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o: IterativeClosestPoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_17) "Building CXX object CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/IterativeClosestPoint.cpp

CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/IterativeClosestPoint.cpp > CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.i

CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/IterativeClosestPoint.cpp -o CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.s

CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.requires

CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.provides: CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.provides

CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.provides.build: CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o


CMakeFiles/01-icp.dir/SegmentCloud.cpp.o: CMakeFiles/01-icp.dir/flags.make
CMakeFiles/01-icp.dir/SegmentCloud.cpp.o: SegmentCloud.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_18) "Building CXX object CMakeFiles/01-icp.dir/SegmentCloud.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/01-icp.dir/SegmentCloud.cpp.o -c /home/misham/MIRI/GPR/01-icp-base/SegmentCloud.cpp

CMakeFiles/01-icp.dir/SegmentCloud.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/01-icp.dir/SegmentCloud.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/misham/MIRI/GPR/01-icp-base/SegmentCloud.cpp > CMakeFiles/01-icp.dir/SegmentCloud.cpp.i

CMakeFiles/01-icp.dir/SegmentCloud.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/01-icp.dir/SegmentCloud.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/misham/MIRI/GPR/01-icp-base/SegmentCloud.cpp -o CMakeFiles/01-icp.dir/SegmentCloud.cpp.s

CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.requires:

.PHONY : CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.requires

CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.provides: CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.requires
	$(MAKE) -f CMakeFiles/01-icp.dir/build.make CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.provides.build
.PHONY : CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.provides

CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.provides.build: CMakeFiles/01-icp.dir/SegmentCloud.cpp.o


# Object files for target 01-icp
01__icp_OBJECTS = \
"CMakeFiles/01-icp.dir/imgui/imgui.cpp.o" \
"CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o" \
"CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o" \
"CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o" \
"CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o" \
"CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o" \
"CMakeFiles/01-icp.dir/gl3w/gl3w.c.o" \
"CMakeFiles/01-icp.dir/PointCloud.cpp.o" \
"CMakeFiles/01-icp.dir/Camera.cpp.o" \
"CMakeFiles/01-icp.dir/Scene.cpp.o" \
"CMakeFiles/01-icp.dir/Shader.cpp.o" \
"CMakeFiles/01-icp.dir/ShaderProgram.cpp.o" \
"CMakeFiles/01-icp.dir/Application.cpp.o" \
"CMakeFiles/01-icp.dir/main.cpp.o" \
"CMakeFiles/01-icp.dir/NormalEstimator.cpp.o" \
"CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o" \
"CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o" \
"CMakeFiles/01-icp.dir/SegmentCloud.cpp.o"

# External object files for target 01-icp
01__icp_EXTERNAL_OBJECTS =

01-icp: CMakeFiles/01-icp.dir/imgui/imgui.cpp.o
01-icp: CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o
01-icp: CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o
01-icp: CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o
01-icp: CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o
01-icp: CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o
01-icp: CMakeFiles/01-icp.dir/gl3w/gl3w.c.o
01-icp: CMakeFiles/01-icp.dir/PointCloud.cpp.o
01-icp: CMakeFiles/01-icp.dir/Camera.cpp.o
01-icp: CMakeFiles/01-icp.dir/Scene.cpp.o
01-icp: CMakeFiles/01-icp.dir/Shader.cpp.o
01-icp: CMakeFiles/01-icp.dir/ShaderProgram.cpp.o
01-icp: CMakeFiles/01-icp.dir/Application.cpp.o
01-icp: CMakeFiles/01-icp.dir/main.cpp.o
01-icp: CMakeFiles/01-icp.dir/NormalEstimator.cpp.o
01-icp: CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o
01-icp: CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o
01-icp: CMakeFiles/01-icp.dir/SegmentCloud.cpp.o
01-icp: CMakeFiles/01-icp.dir/build.make
01-icp: /usr/lib/x86_64-linux-gnu/libGL.so
01-icp: /usr/lib/x86_64-linux-gnu/libGLU.so
01-icp: /usr/lib/x86_64-linux-gnu/libglut.so
01-icp: /usr/lib/x86_64-linux-gnu/libXi.so
01-icp: CMakeFiles/01-icp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/misham/MIRI/GPR/01-icp-base/CMakeFiles --progress-num=$(CMAKE_PROGRESS_19) "Linking CXX executable 01-icp"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/01-icp.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/01-icp.dir/build: 01-icp

.PHONY : CMakeFiles/01-icp.dir/build

CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/imgui/imgui.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/imgui/imgui_demo.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/imgui/imgui_draw.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/imgui/imgui_widgets.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/imgui/imgui_impl_opengl3.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/imgui/imgui_impl_glut.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/gl3w/gl3w.c.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/PointCloud.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/Camera.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/Scene.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/Shader.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/ShaderProgram.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/Application.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/main.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/NormalEstimator.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/NearestNeighbors.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/IterativeClosestPoint.cpp.o.requires
CMakeFiles/01-icp.dir/requires: CMakeFiles/01-icp.dir/SegmentCloud.cpp.o.requires

.PHONY : CMakeFiles/01-icp.dir/requires

CMakeFiles/01-icp.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/01-icp.dir/cmake_clean.cmake
.PHONY : CMakeFiles/01-icp.dir/clean

CMakeFiles/01-icp.dir/depend:
	cd /home/misham/MIRI/GPR/01-icp-base && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/misham/MIRI/GPR/01-icp-base /home/misham/MIRI/GPR/01-icp-base /home/misham/MIRI/GPR/01-icp-base /home/misham/MIRI/GPR/01-icp-base /home/misham/MIRI/GPR/01-icp-base/CMakeFiles/01-icp.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/01-icp.dir/depend


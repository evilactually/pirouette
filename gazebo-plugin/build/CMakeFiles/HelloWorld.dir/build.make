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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/vlsh/Sources/pirouette/gazebo-plugin

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/vlsh/Sources/pirouette/gazebo-plugin/build

# Include any dependencies generated for this target.
include CMakeFiles/HelloWorld.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/HelloWorld.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/HelloWorld.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/HelloWorld.dir/flags.make

CMakeFiles/HelloWorld.dir/HelloWorld.cc.o: CMakeFiles/HelloWorld.dir/flags.make
CMakeFiles/HelloWorld.dir/HelloWorld.cc.o: /home/vlsh/Sources/pirouette/gazebo-plugin/HelloWorld.cc
CMakeFiles/HelloWorld.dir/HelloWorld.cc.o: CMakeFiles/HelloWorld.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/vlsh/Sources/pirouette/gazebo-plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/HelloWorld.dir/HelloWorld.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/HelloWorld.dir/HelloWorld.cc.o -MF CMakeFiles/HelloWorld.dir/HelloWorld.cc.o.d -o CMakeFiles/HelloWorld.dir/HelloWorld.cc.o -c /home/vlsh/Sources/pirouette/gazebo-plugin/HelloWorld.cc

CMakeFiles/HelloWorld.dir/HelloWorld.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/HelloWorld.dir/HelloWorld.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/vlsh/Sources/pirouette/gazebo-plugin/HelloWorld.cc > CMakeFiles/HelloWorld.dir/HelloWorld.cc.i

CMakeFiles/HelloWorld.dir/HelloWorld.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/HelloWorld.dir/HelloWorld.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/vlsh/Sources/pirouette/gazebo-plugin/HelloWorld.cc -o CMakeFiles/HelloWorld.dir/HelloWorld.cc.s

# Object files for target HelloWorld
HelloWorld_OBJECTS = \
"CMakeFiles/HelloWorld.dir/HelloWorld.cc.o"

# External object files for target HelloWorld
HelloWorld_EXTERNAL_OBJECTS =

libHelloWorld.so: CMakeFiles/HelloWorld.dir/HelloWorld.cc.o
libHelloWorld.so: CMakeFiles/HelloWorld.dir/build.make
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-sim7.so.7.5.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-fuel_tools8.so.8.0.2
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-gui7.so.7.2.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2-loader.so.2.0.1
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5QuickControls2.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.12.8
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-physics6.so.6.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2.so.2.0.1
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-rendering7.so.7.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5-profiler.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5-events.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5-av.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5-io.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5-geospatial.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5-graphics.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-common5.so.5.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-transport12-parameters.so.12.2.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-transport12.so.12.2.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-msgs9.so.9.4.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libsdformat13.so.13.5.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-math7.so.7.1.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libgz-utils2.so.2.0.0
libHelloWorld.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libHelloWorld.so: CMakeFiles/HelloWorld.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/vlsh/Sources/pirouette/gazebo-plugin/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libHelloWorld.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/HelloWorld.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/HelloWorld.dir/build: libHelloWorld.so
.PHONY : CMakeFiles/HelloWorld.dir/build

CMakeFiles/HelloWorld.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/HelloWorld.dir/cmake_clean.cmake
.PHONY : CMakeFiles/HelloWorld.dir/clean

CMakeFiles/HelloWorld.dir/depend:
	cd /home/vlsh/Sources/pirouette/gazebo-plugin/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/vlsh/Sources/pirouette/gazebo-plugin /home/vlsh/Sources/pirouette/gazebo-plugin /home/vlsh/Sources/pirouette/gazebo-plugin/build /home/vlsh/Sources/pirouette/gazebo-plugin/build /home/vlsh/Sources/pirouette/gazebo-plugin/build/CMakeFiles/HelloWorld.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/HelloWorld.dir/depend


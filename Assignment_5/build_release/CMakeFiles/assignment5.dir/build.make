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
CMAKE_SOURCE_DIR = /home/ameduri/course_work/computer-graphics/Assignment_5

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ameduri/course_work/computer-graphics/Assignment_5/build_release

# Include any dependencies generated for this target.
include CMakeFiles/assignment5.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/assignment5.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/assignment5.dir/flags.make

CMakeFiles/assignment5.dir/src/main.cpp.o: CMakeFiles/assignment5.dir/flags.make
CMakeFiles/assignment5.dir/src/main.cpp.o: ../src/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ameduri/course_work/computer-graphics/Assignment_5/build_release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/assignment5.dir/src/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/assignment5.dir/src/main.cpp.o -c /home/ameduri/course_work/computer-graphics/Assignment_5/src/main.cpp

CMakeFiles/assignment5.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignment5.dir/src/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ameduri/course_work/computer-graphics/Assignment_5/src/main.cpp > CMakeFiles/assignment5.dir/src/main.cpp.i

CMakeFiles/assignment5.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignment5.dir/src/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ameduri/course_work/computer-graphics/Assignment_5/src/main.cpp -o CMakeFiles/assignment5.dir/src/main.cpp.s

CMakeFiles/assignment5.dir/src/main.cpp.o.requires:

.PHONY : CMakeFiles/assignment5.dir/src/main.cpp.o.requires

CMakeFiles/assignment5.dir/src/main.cpp.o.provides: CMakeFiles/assignment5.dir/src/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/assignment5.dir/build.make CMakeFiles/assignment5.dir/src/main.cpp.o.provides.build
.PHONY : CMakeFiles/assignment5.dir/src/main.cpp.o.provides

CMakeFiles/assignment5.dir/src/main.cpp.o.provides.build: CMakeFiles/assignment5.dir/src/main.cpp.o


CMakeFiles/assignment5.dir/src/raster.cpp.o: CMakeFiles/assignment5.dir/flags.make
CMakeFiles/assignment5.dir/src/raster.cpp.o: ../src/raster.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ameduri/course_work/computer-graphics/Assignment_5/build_release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/assignment5.dir/src/raster.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/assignment5.dir/src/raster.cpp.o -c /home/ameduri/course_work/computer-graphics/Assignment_5/src/raster.cpp

CMakeFiles/assignment5.dir/src/raster.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/assignment5.dir/src/raster.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ameduri/course_work/computer-graphics/Assignment_5/src/raster.cpp > CMakeFiles/assignment5.dir/src/raster.cpp.i

CMakeFiles/assignment5.dir/src/raster.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/assignment5.dir/src/raster.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ameduri/course_work/computer-graphics/Assignment_5/src/raster.cpp -o CMakeFiles/assignment5.dir/src/raster.cpp.s

CMakeFiles/assignment5.dir/src/raster.cpp.o.requires:

.PHONY : CMakeFiles/assignment5.dir/src/raster.cpp.o.requires

CMakeFiles/assignment5.dir/src/raster.cpp.o.provides: CMakeFiles/assignment5.dir/src/raster.cpp.o.requires
	$(MAKE) -f CMakeFiles/assignment5.dir/build.make CMakeFiles/assignment5.dir/src/raster.cpp.o.provides.build
.PHONY : CMakeFiles/assignment5.dir/src/raster.cpp.o.provides

CMakeFiles/assignment5.dir/src/raster.cpp.o.provides.build: CMakeFiles/assignment5.dir/src/raster.cpp.o


# Object files for target assignment5
assignment5_OBJECTS = \
"CMakeFiles/assignment5.dir/src/main.cpp.o" \
"CMakeFiles/assignment5.dir/src/raster.cpp.o"

# External object files for target assignment5
assignment5_EXTERNAL_OBJECTS =

assignment5: CMakeFiles/assignment5.dir/src/main.cpp.o
assignment5: CMakeFiles/assignment5.dir/src/raster.cpp.o
assignment5: CMakeFiles/assignment5.dir/build.make
assignment5: CMakeFiles/assignment5.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ameduri/course_work/computer-graphics/Assignment_5/build_release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable assignment5"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/assignment5.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/assignment5.dir/build: assignment5

.PHONY : CMakeFiles/assignment5.dir/build

CMakeFiles/assignment5.dir/requires: CMakeFiles/assignment5.dir/src/main.cpp.o.requires
CMakeFiles/assignment5.dir/requires: CMakeFiles/assignment5.dir/src/raster.cpp.o.requires

.PHONY : CMakeFiles/assignment5.dir/requires

CMakeFiles/assignment5.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/assignment5.dir/cmake_clean.cmake
.PHONY : CMakeFiles/assignment5.dir/clean

CMakeFiles/assignment5.dir/depend:
	cd /home/ameduri/course_work/computer-graphics/Assignment_5/build_release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ameduri/course_work/computer-graphics/Assignment_5 /home/ameduri/course_work/computer-graphics/Assignment_5 /home/ameduri/course_work/computer-graphics/Assignment_5/build_release /home/ameduri/course_work/computer-graphics/Assignment_5/build_release /home/ameduri/course_work/computer-graphics/Assignment_5/build_release/CMakeFiles/assignment5.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/assignment5.dir/depend


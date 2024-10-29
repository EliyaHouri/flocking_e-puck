# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

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
CMAKE_SOURCE_DIR = /home/eliyahu/Desktop/e-puck2/examples/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eliyahu/Desktop/e-puck2/examples/src/experiments

# Include any dependencies generated for this target.
include controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/compiler_depend.make

# Include the progress variables for this target.
include controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/progress.make

# Include the compile flags for this target's objects.
include controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/flags.make

controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o: controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/flags.make
controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o: ../controllers/epuck2_sensors/epuck2_sensors.cpp
controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o: controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/eliyahu/Desktop/e-puck2/examples/src/experiments/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o"
	cd /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o -MF CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o.d -o CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o -c /home/eliyahu/Desktop/e-puck2/examples/src/controllers/epuck2_sensors/epuck2_sensors.cpp

controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/epuck2_sensors.dir/epuck2_sensors.i"
	cd /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/eliyahu/Desktop/e-puck2/examples/src/controllers/epuck2_sensors/epuck2_sensors.cpp > CMakeFiles/epuck2_sensors.dir/epuck2_sensors.i

controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/epuck2_sensors.dir/epuck2_sensors.s"
	cd /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/eliyahu/Desktop/e-puck2/examples/src/controllers/epuck2_sensors/epuck2_sensors.cpp -o CMakeFiles/epuck2_sensors.dir/epuck2_sensors.s

# Object files for target epuck2_sensors
epuck2_sensors_OBJECTS = \
"CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o"

# External object files for target epuck2_sensors
epuck2_sensors_EXTERNAL_OBJECTS =

controllers/epuck2_sensors/libepuck2_sensors.so: controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/epuck2_sensors.o
controllers/epuck2_sensors/libepuck2_sensors.so: controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/build.make
controllers/epuck2_sensors/libepuck2_sensors.so: controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/eliyahu/Desktop/e-puck2/examples/src/experiments/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared module libepuck2_sensors.so"
	cd /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/epuck2_sensors.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/build: controllers/epuck2_sensors/libepuck2_sensors.so
.PHONY : controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/build

controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/clean:
	cd /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors && $(CMAKE_COMMAND) -P CMakeFiles/epuck2_sensors.dir/cmake_clean.cmake
.PHONY : controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/clean

controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/depend:
	cd /home/eliyahu/Desktop/e-puck2/examples/src/experiments && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eliyahu/Desktop/e-puck2/examples/src /home/eliyahu/Desktop/e-puck2/examples/src/controllers/epuck2_sensors /home/eliyahu/Desktop/e-puck2/examples/src/experiments /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors /home/eliyahu/Desktop/e-puck2/examples/src/experiments/controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : controllers/epuck2_sensors/CMakeFiles/epuck2_sensors.dir/depend


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
CMAKE_SOURCE_DIR = /home/eliyahu/Desktop/e-puck2/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/eliyahu/Desktop/e-puck2/build

# Utility rule file for argos3plugin_simulator_epuck2_autogen.

# Include any custom commands dependencies for this target.
include plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/compiler_depend.make

# Include the progress variables for this target.
include plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/progress.make

plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/eliyahu/Desktop/e-puck2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Automatic MOC for target argos3plugin_simulator_epuck2"
	cd /home/eliyahu/Desktop/e-puck2/build/plugins/robots/e-puck2 && /usr/bin/cmake -E cmake_autogen /home/eliyahu/Desktop/e-puck2/build/plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/AutogenInfo.json Release

argos3plugin_simulator_epuck2_autogen: plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen
argos3plugin_simulator_epuck2_autogen: plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/build.make
.PHONY : argos3plugin_simulator_epuck2_autogen

# Rule to build all files generated by this target.
plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/build: argos3plugin_simulator_epuck2_autogen
.PHONY : plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/build

plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/clean:
	cd /home/eliyahu/Desktop/e-puck2/build/plugins/robots/e-puck2 && $(CMAKE_COMMAND) -P CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/cmake_clean.cmake
.PHONY : plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/clean

plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/depend:
	cd /home/eliyahu/Desktop/e-puck2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/eliyahu/Desktop/e-puck2/src /home/eliyahu/Desktop/e-puck2/src/plugins/robots/e-puck2 /home/eliyahu/Desktop/e-puck2/build /home/eliyahu/Desktop/e-puck2/build/plugins/robots/e-puck2 /home/eliyahu/Desktop/e-puck2/build/plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : plugins/robots/e-puck2/CMakeFiles/argos3plugin_simulator_epuck2_autogen.dir/depend


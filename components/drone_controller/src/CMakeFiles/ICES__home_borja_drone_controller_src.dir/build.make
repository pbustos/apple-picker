# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

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
CMAKE_SOURCE_DIR = /home/borja/drone_controller/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/borja/drone_controller/src

# Utility rule file for ICES__home_borja_drone_controller_src.

# Include the progress variables for this target.
include CMakeFiles/ICES__home_borja_drone_controller_src.dir/progress.make

ICES__home_borja_drone_controller_src: CMakeFiles/ICES__home_borja_drone_controller_src.dir/build.make
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating CommonBehavior.ice from /home/borja/robocomp/interfaces/IDSLs/CommonBehavior.idsl"
	robocompdsl /home/borja/robocomp/interfaces/IDSLs/CommonBehavior.idsl /home/borja/drone_controller/src/CommonBehavior.ice
	robocompdsl /home/borja/robocomp/interfaces/IDSLs/CommonBehavior.idsl CommonBehavior.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating CameraRGBDSimple.ice from /home/borja/robocomp/interfaces/IDSLs/CameraRGBDSimple.idsl"
	robocompdsl /home/borja/robocomp/interfaces/IDSLs/CameraRGBDSimple.idsl /home/borja/drone_controller/src/CameraRGBDSimple.ice
	robocompdsl /home/borja/robocomp/interfaces/IDSLs/CameraRGBDSimple.idsl CameraRGBDSimple.ice
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating JoystickAdapter.ice from /home/borja/robocomp/interfaces/IDSLs/JoystickAdapter.idsl"
	robocompdsl /home/borja/robocomp/interfaces/IDSLs/JoystickAdapter.idsl /home/borja/drone_controller/src/JoystickAdapter.ice
	robocompdsl /home/borja/robocomp/interfaces/IDSLs/JoystickAdapter.idsl JoystickAdapter.ice
.PHONY : ICES__home_borja_drone_controller_src

# Rule to build all files generated by this target.
CMakeFiles/ICES__home_borja_drone_controller_src.dir/build: ICES__home_borja_drone_controller_src

.PHONY : CMakeFiles/ICES__home_borja_drone_controller_src.dir/build

CMakeFiles/ICES__home_borja_drone_controller_src.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ICES__home_borja_drone_controller_src.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ICES__home_borja_drone_controller_src.dir/clean

CMakeFiles/ICES__home_borja_drone_controller_src.dir/depend:
	cd /home/borja/drone_controller/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/borja/drone_controller/src /home/borja/drone_controller/src /home/borja/drone_controller/src /home/borja/drone_controller/src /home/borja/drone_controller/src/CMakeFiles/ICES__home_borja_drone_controller_src.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ICES__home_borja_drone_controller_src.dir/depend


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
CMAKE_SOURCE_DIR = /root/git/scratchpad/csc496

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /root/git/scratchpad/csc496/runners

# Include any dependencies generated for this target.
include CMakeFiles/run_leader_franka.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/run_leader_franka.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/run_leader_franka.dir/flags.make

CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.o: CMakeFiles/run_leader_franka.dir/flags.make
CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.o: teleop/leader_franka.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/root/git/scratchpad/csc496/runners/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.o -c /root/git/scratchpad/csc496/runners/teleop/leader_franka.cpp

CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /root/git/scratchpad/csc496/runners/teleop/leader_franka.cpp > CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.i

CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /root/git/scratchpad/csc496/runners/teleop/leader_franka.cpp -o CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.s

# Object files for target run_leader_franka
run_leader_franka_OBJECTS = \
"CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.o"

# External object files for target run_leader_franka
run_leader_franka_EXTERNAL_OBJECTS =

run_leader_franka: CMakeFiles/run_leader_franka.dir/teleop/leader_franka.cpp.o
run_leader_franka: CMakeFiles/run_leader_franka.dir/build.make
run_leader_franka: libleader.a
run_leader_franka: libnetwork_client.a
run_leader_franka: libfranka_common.a
run_leader_franka: /usr/local/lib/libfranka.so.0.9.2
run_leader_franka: /usr/lib/x86_64-linux-gnu/libboost_system.so.1.71.0
run_leader_franka: /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.71.0
run_leader_franka: /usr/lib/x86_64-linux-gnu/libboost_atomic.so.1.71.0
run_leader_franka: CMakeFiles/run_leader_franka.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/root/git/scratchpad/csc496/runners/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable run_leader_franka"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/run_leader_franka.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/run_leader_franka.dir/build: run_leader_franka

.PHONY : CMakeFiles/run_leader_franka.dir/build

CMakeFiles/run_leader_franka.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_leader_franka.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_leader_franka.dir/clean

CMakeFiles/run_leader_franka.dir/depend:
	cd /root/git/scratchpad/csc496/runners && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /root/git/scratchpad/csc496 /root/git/scratchpad/csc496 /root/git/scratchpad/csc496/runners /root/git/scratchpad/csc496/runners /root/git/scratchpad/csc496/runners/CMakeFiles/run_leader_franka.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_leader_franka.dir/depend


# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/avalenzu/drc/software/perception/lidar-plancheck

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build

# Include any dependencies generated for this target.
include src/stream/CMakeFiles/drc-stream.dir/depend.make

# Include the progress variables for this target.
include src/stream/CMakeFiles/drc-stream.dir/progress.make

# Include the compile flags for this target's objects.
include src/stream/CMakeFiles/drc-stream.dir/flags.make

src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o: src/stream/CMakeFiles/drc-stream.dir/flags.make
src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o: ../src/stream/stream.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o"
	cd /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream && /usr/lib/ccache/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/drc-stream.dir/stream.cpp.o -c /home/avalenzu/drc/software/perception/lidar-plancheck/src/stream/stream.cpp

src/stream/CMakeFiles/drc-stream.dir/stream.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drc-stream.dir/stream.cpp.i"
	cd /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/avalenzu/drc/software/perception/lidar-plancheck/src/stream/stream.cpp > CMakeFiles/drc-stream.dir/stream.cpp.i

src/stream/CMakeFiles/drc-stream.dir/stream.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drc-stream.dir/stream.cpp.s"
	cd /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream && /usr/lib/ccache/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/avalenzu/drc/software/perception/lidar-plancheck/src/stream/stream.cpp -o CMakeFiles/drc-stream.dir/stream.cpp.s

src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.requires:
.PHONY : src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.requires

src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.provides: src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.requires
	$(MAKE) -f src/stream/CMakeFiles/drc-stream.dir/build.make src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.provides.build
.PHONY : src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.provides

src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.provides.build: src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o

# Object files for target drc-stream
drc__stream_OBJECTS = \
"CMakeFiles/drc-stream.dir/stream.cpp.o"

# External object files for target drc-stream
drc__stream_EXTERNAL_OBJECTS =

bin/drc-stream: src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o
bin/drc-stream: src/stream/CMakeFiles/drc-stream.dir/build.make
bin/drc-stream: src/stream/CMakeFiles/drc-stream.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ../../bin/drc-stream"
	cd /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drc-stream.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/stream/CMakeFiles/drc-stream.dir/build: bin/drc-stream
.PHONY : src/stream/CMakeFiles/drc-stream.dir/build

src/stream/CMakeFiles/drc-stream.dir/requires: src/stream/CMakeFiles/drc-stream.dir/stream.cpp.o.requires
.PHONY : src/stream/CMakeFiles/drc-stream.dir/requires

src/stream/CMakeFiles/drc-stream.dir/clean:
	cd /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream && $(CMAKE_COMMAND) -P CMakeFiles/drc-stream.dir/cmake_clean.cmake
.PHONY : src/stream/CMakeFiles/drc-stream.dir/clean

src/stream/CMakeFiles/drc-stream.dir/depend:
	cd /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/avalenzu/drc/software/perception/lidar-plancheck /home/avalenzu/drc/software/perception/lidar-plancheck/src/stream /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream /home/avalenzu/drc/software/perception/lidar-plancheck/pod-build/src/stream/CMakeFiles/drc-stream.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/stream/CMakeFiles/drc-stream.dir/depend


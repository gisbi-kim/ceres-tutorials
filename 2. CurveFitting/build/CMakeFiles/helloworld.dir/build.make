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
CMAKE_SOURCE_DIR = "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting"

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build"

# Include any dependencies generated for this target.
include CMakeFiles/helloworld.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/helloworld.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/helloworld.dir/flags.make

CMakeFiles/helloworld.dir/main.cpp.o: CMakeFiles/helloworld.dir/flags.make
CMakeFiles/helloworld.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir="/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/helloworld.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/helloworld.dir/main.cpp.o -c "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/main.cpp"

CMakeFiles/helloworld.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/helloworld.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/main.cpp" > CMakeFiles/helloworld.dir/main.cpp.i

CMakeFiles/helloworld.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/helloworld.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/main.cpp" -o CMakeFiles/helloworld.dir/main.cpp.s

CMakeFiles/helloworld.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/helloworld.dir/main.cpp.o.requires

CMakeFiles/helloworld.dir/main.cpp.o.provides: CMakeFiles/helloworld.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/helloworld.dir/build.make CMakeFiles/helloworld.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/helloworld.dir/main.cpp.o.provides

CMakeFiles/helloworld.dir/main.cpp.o.provides.build: CMakeFiles/helloworld.dir/main.cpp.o


# Object files for target helloworld
helloworld_OBJECTS = \
"CMakeFiles/helloworld.dir/main.cpp.o"

# External object files for target helloworld
helloworld_EXTERNAL_OBJECTS =

helloworld: CMakeFiles/helloworld.dir/main.cpp.o
helloworld: CMakeFiles/helloworld.dir/build.make
helloworld: /usr/local/lib/libceres.a
helloworld: /usr/lib/x86_64-linux-gnu/libglog.so
helloworld: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.1
helloworld: /usr/lib/x86_64-linux-gnu/libspqr.so
helloworld: /usr/lib/x86_64-linux-gnu/libtbbmalloc.so
helloworld: /usr/lib/x86_64-linux-gnu/libtbb.so
helloworld: /usr/lib/x86_64-linux-gnu/libcholmod.so
helloworld: /usr/lib/x86_64-linux-gnu/libccolamd.so
helloworld: /usr/lib/x86_64-linux-gnu/libcamd.so
helloworld: /usr/lib/x86_64-linux-gnu/libcolamd.so
helloworld: /usr/lib/x86_64-linux-gnu/libamd.so
helloworld: /usr/lib/x86_64-linux-gnu/liblapack.so
helloworld: /usr/lib/x86_64-linux-gnu/libf77blas.so
helloworld: /usr/lib/x86_64-linux-gnu/libatlas.so
helloworld: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
helloworld: /usr/lib/x86_64-linux-gnu/librt.so
helloworld: /usr/local/lib/libmetis.so
helloworld: /usr/lib/x86_64-linux-gnu/libcxsparse.so
helloworld: /usr/lib/x86_64-linux-gnu/liblapack.so
helloworld: /usr/lib/x86_64-linux-gnu/libf77blas.so
helloworld: /usr/lib/x86_64-linux-gnu/libatlas.so
helloworld: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
helloworld: /usr/lib/x86_64-linux-gnu/librt.so
helloworld: /usr/local/lib/libmetis.so
helloworld: /usr/lib/x86_64-linux-gnu/libcxsparse.so
helloworld: CMakeFiles/helloworld.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir="/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build/CMakeFiles" --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable helloworld"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/helloworld.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/helloworld.dir/build: helloworld

.PHONY : CMakeFiles/helloworld.dir/build

CMakeFiles/helloworld.dir/requires: CMakeFiles/helloworld.dir/main.cpp.o.requires

.PHONY : CMakeFiles/helloworld.dir/requires

CMakeFiles/helloworld.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/helloworld.dir/cmake_clean.cmake
.PHONY : CMakeFiles/helloworld.dir/clean

CMakeFiles/helloworld.dir/depend:
	cd "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build" && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting" "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting" "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build" "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build" "/home/user/Downloads/ceres/tuto1/ceres-tutorials/2. CurveFitting/build/CMakeFiles/helloworld.dir/DependInfo.cmake" --color=$(COLOR)
.PHONY : CMakeFiles/helloworld.dir/depend

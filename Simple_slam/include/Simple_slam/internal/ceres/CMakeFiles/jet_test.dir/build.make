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
CMAKE_SOURCE_DIR = /home/cona/ceres-solver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/cona/ceres-solver

# Include any dependencies generated for this target.
include internal/ceres/CMakeFiles/jet_test.dir/depend.make

# Include the progress variables for this target.
include internal/ceres/CMakeFiles/jet_test.dir/progress.make

# Include the compile flags for this target's objects.
include internal/ceres/CMakeFiles/jet_test.dir/flags.make

internal/ceres/CMakeFiles/jet_test.dir/jet_test.cc.o: internal/ceres/CMakeFiles/jet_test.dir/flags.make
internal/ceres/CMakeFiles/jet_test.dir/jet_test.cc.o: internal/ceres/jet_test.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/cona/ceres-solver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object internal/ceres/CMakeFiles/jet_test.dir/jet_test.cc.o"
	cd /home/cona/ceres-solver/internal/ceres && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/jet_test.dir/jet_test.cc.o -c /home/cona/ceres-solver/internal/ceres/jet_test.cc

internal/ceres/CMakeFiles/jet_test.dir/jet_test.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/jet_test.dir/jet_test.cc.i"
	cd /home/cona/ceres-solver/internal/ceres && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/cona/ceres-solver/internal/ceres/jet_test.cc > CMakeFiles/jet_test.dir/jet_test.cc.i

internal/ceres/CMakeFiles/jet_test.dir/jet_test.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/jet_test.dir/jet_test.cc.s"
	cd /home/cona/ceres-solver/internal/ceres && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/cona/ceres-solver/internal/ceres/jet_test.cc -o CMakeFiles/jet_test.dir/jet_test.cc.s

# Object files for target jet_test
jet_test_OBJECTS = \
"CMakeFiles/jet_test.dir/jet_test.cc.o"

# External object files for target jet_test
jet_test_EXTERNAL_OBJECTS =

bin/jet_test: internal/ceres/CMakeFiles/jet_test.dir/jet_test.cc.o
bin/jet_test: internal/ceres/CMakeFiles/jet_test.dir/build.make
bin/jet_test: lib/libgtest.a
bin/jet_test: lib/libtest_util.a
bin/jet_test: lib/libceres.a
bin/jet_test: /usr/lib/x86_64-linux-gnu/libspqr.so
bin/jet_test: lib/libgtest.a
bin/jet_test: lib/libceres.a
bin/jet_test: /usr/lib/x86_64-linux-gnu/libspqr.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libtbb.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libglog.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libcholmod.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libamd.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libcamd.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libccolamd.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libcolamd.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/liblapack.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libf77blas.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libatlas.so
bin/jet_test: /usr/lib/x86_64-linux-gnu/libgflags.so.2.2.2
bin/jet_test: internal/ceres/CMakeFiles/jet_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/cona/ceres-solver/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../../bin/jet_test"
	cd /home/cona/ceres-solver/internal/ceres && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/jet_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
internal/ceres/CMakeFiles/jet_test.dir/build: bin/jet_test

.PHONY : internal/ceres/CMakeFiles/jet_test.dir/build

internal/ceres/CMakeFiles/jet_test.dir/clean:
	cd /home/cona/ceres-solver/internal/ceres && $(CMAKE_COMMAND) -P CMakeFiles/jet_test.dir/cmake_clean.cmake
.PHONY : internal/ceres/CMakeFiles/jet_test.dir/clean

internal/ceres/CMakeFiles/jet_test.dir/depend:
	cd /home/cona/ceres-solver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/cona/ceres-solver /home/cona/ceres-solver/internal/ceres /home/cona/ceres-solver /home/cona/ceres-solver/internal/ceres /home/cona/ceres-solver/internal/ceres/CMakeFiles/jet_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : internal/ceres/CMakeFiles/jet_test.dir/depend


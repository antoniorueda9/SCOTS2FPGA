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
CMAKE_SOURCE_DIR = /home/antonio/Escritorio/Project_2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/antonio/Escritorio/Project_2/build

# Include any dependencies generated for this target.
include src/CMakeFiles/wrapper.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/wrapper.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/wrapper.dir/flags.make

src/CMakeFiles/wrapper.dir/wrapper.cc.o: src/CMakeFiles/wrapper.dir/flags.make
src/CMakeFiles/wrapper.dir/wrapper.cc.o: ../src/wrapper.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/antonio/Escritorio/Project_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/wrapper.dir/wrapper.cc.o"
	cd /home/antonio/Escritorio/Project_2/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/wrapper.dir/wrapper.cc.o -c /home/antonio/Escritorio/Project_2/src/wrapper.cc

src/CMakeFiles/wrapper.dir/wrapper.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/wrapper.dir/wrapper.cc.i"
	cd /home/antonio/Escritorio/Project_2/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/antonio/Escritorio/Project_2/src/wrapper.cc > CMakeFiles/wrapper.dir/wrapper.cc.i

src/CMakeFiles/wrapper.dir/wrapper.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/wrapper.dir/wrapper.cc.s"
	cd /home/antonio/Escritorio/Project_2/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/antonio/Escritorio/Project_2/src/wrapper.cc -o CMakeFiles/wrapper.dir/wrapper.cc.s

src/CMakeFiles/wrapper.dir/wrapper.cc.o.requires:

.PHONY : src/CMakeFiles/wrapper.dir/wrapper.cc.o.requires

src/CMakeFiles/wrapper.dir/wrapper.cc.o.provides: src/CMakeFiles/wrapper.dir/wrapper.cc.o.requires
	$(MAKE) -f src/CMakeFiles/wrapper.dir/build.make src/CMakeFiles/wrapper.dir/wrapper.cc.o.provides.build
.PHONY : src/CMakeFiles/wrapper.dir/wrapper.cc.o.provides

src/CMakeFiles/wrapper.dir/wrapper.cc.o.provides.build: src/CMakeFiles/wrapper.dir/wrapper.cc.o


# Object files for target wrapper
wrapper_OBJECTS = \
"CMakeFiles/wrapper.dir/wrapper.cc.o"

# External object files for target wrapper
wrapper_EXTERNAL_OBJECTS =

src/wrapper: src/CMakeFiles/wrapper.dir/wrapper.cc.o
src/wrapper: src/CMakeFiles/wrapper.dir/build.make
src/wrapper: src/CMakeFiles/wrapper.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/antonio/Escritorio/Project_2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable wrapper"
	cd /home/antonio/Escritorio/Project_2/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/wrapper.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/wrapper.dir/build: src/wrapper

.PHONY : src/CMakeFiles/wrapper.dir/build

src/CMakeFiles/wrapper.dir/requires: src/CMakeFiles/wrapper.dir/wrapper.cc.o.requires

.PHONY : src/CMakeFiles/wrapper.dir/requires

src/CMakeFiles/wrapper.dir/clean:
	cd /home/antonio/Escritorio/Project_2/build/src && $(CMAKE_COMMAND) -P CMakeFiles/wrapper.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/wrapper.dir/clean

src/CMakeFiles/wrapper.dir/depend:
	cd /home/antonio/Escritorio/Project_2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/antonio/Escritorio/Project_2 /home/antonio/Escritorio/Project_2/src /home/antonio/Escritorio/Project_2/build /home/antonio/Escritorio/Project_2/build/src /home/antonio/Escritorio/Project_2/build/src/CMakeFiles/wrapper.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/wrapper.dir/depend

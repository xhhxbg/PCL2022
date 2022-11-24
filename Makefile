# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


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
CMAKE_SOURCE_DIR = /home/yyf/Documents/SRTP_ICP2022

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yyf/Documents/SRTP_ICP2022

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/ccmake -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/yyf/Documents/SRTP_ICP2022/CMakeFiles /home/yyf/Documents/SRTP_ICP2022/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/yyf/Documents/SRTP_ICP2022/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named ICP_msg_handler

# Build rule for target.
ICP_msg_handler: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ICP_msg_handler
.PHONY : ICP_msg_handler

# fast build rule for target.
ICP_msg_handler/fast:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/build
.PHONY : ICP_msg_handler/fast

src/ROS_msg_handler.o: src/ROS_msg_handler.cpp.o

.PHONY : src/ROS_msg_handler.o

# target to build an object file
src/ROS_msg_handler.cpp.o:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/src/ROS_msg_handler.cpp.o
.PHONY : src/ROS_msg_handler.cpp.o

src/ROS_msg_handler.i: src/ROS_msg_handler.cpp.i

.PHONY : src/ROS_msg_handler.i

# target to preprocess a source file
src/ROS_msg_handler.cpp.i:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/src/ROS_msg_handler.cpp.i
.PHONY : src/ROS_msg_handler.cpp.i

src/ROS_msg_handler.s: src/ROS_msg_handler.cpp.s

.PHONY : src/ROS_msg_handler.s

# target to generate assembly for a file
src/ROS_msg_handler.cpp.s:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/src/ROS_msg_handler.cpp.s
.PHONY : src/ROS_msg_handler.cpp.s

src/ROSmsg.o: src/ROSmsg.cpp.o

.PHONY : src/ROSmsg.o

# target to build an object file
src/ROSmsg.cpp.o:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/src/ROSmsg.cpp.o
.PHONY : src/ROSmsg.cpp.o

src/ROSmsg.i: src/ROSmsg.cpp.i

.PHONY : src/ROSmsg.i

# target to preprocess a source file
src/ROSmsg.cpp.i:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/src/ROSmsg.cpp.i
.PHONY : src/ROSmsg.cpp.i

src/ROSmsg.s: src/ROSmsg.cpp.s

.PHONY : src/ROSmsg.s

# target to generate assembly for a file
src/ROSmsg.cpp.s:
	$(MAKE) -f CMakeFiles/ICP_msg_handler.dir/build.make CMakeFiles/ICP_msg_handler.dir/src/ROSmsg.cpp.s
.PHONY : src/ROSmsg.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... edit_cache"
	@echo "... ICP_msg_handler"
	@echo "... src/ROS_msg_handler.o"
	@echo "... src/ROS_msg_handler.i"
	@echo "... src/ROS_msg_handler.s"
	@echo "... src/ROSmsg.o"
	@echo "... src/ROSmsg.i"
	@echo "... src/ROSmsg.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.4

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/projet_tristan/projet

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/projet_tristan/projet

# Include any dependencies generated for this target.
include CMakeFiles/projet_imu.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/projet_imu.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/projet_imu.dir/flags.make

CMakeFiles/projet_imu.dir/main.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/main.cpp.o: main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/projet_imu.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/main.cpp.o -c /home/pi/projet_tristan/projet/main.cpp

CMakeFiles/projet_imu.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/main.cpp > CMakeFiles/projet_imu.dir/main.cpp.i

CMakeFiles/projet_imu.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/main.cpp -o CMakeFiles/projet_imu.dir/main.cpp.s

CMakeFiles/projet_imu.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/main.cpp.o.requires

CMakeFiles/projet_imu.dir/main.cpp.o.provides: CMakeFiles/projet_imu.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/main.cpp.o.provides

CMakeFiles/projet_imu.dir/main.cpp.o.provides.build: CMakeFiles/projet_imu.dir/main.cpp.o


CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o: IMU_Sensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o -c /home/pi/projet_tristan/projet/IMU_Sensor.cpp

CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/IMU_Sensor.cpp > CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.i

CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/IMU_Sensor.cpp -o CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.s

CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.requires

CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.provides: CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.provides

CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.provides.build: CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o


CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o: EMG_Sensor.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o -c /home/pi/projet_tristan/projet/EMG_Sensor.cpp

CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/EMG_Sensor.cpp > CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.i

CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/EMG_Sensor.cpp -o CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.s

CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.requires

CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.provides: CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.provides

CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.provides.build: CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o


CMakeFiles/projet_imu.dir/Control_JACO.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/Control_JACO.cpp.o: Control_JACO.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/projet_imu.dir/Control_JACO.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/Control_JACO.cpp.o -c /home/pi/projet_tristan/projet/Control_JACO.cpp

CMakeFiles/projet_imu.dir/Control_JACO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/Control_JACO.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/Control_JACO.cpp > CMakeFiles/projet_imu.dir/Control_JACO.cpp.i

CMakeFiles/projet_imu.dir/Control_JACO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/Control_JACO.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/Control_JACO.cpp -o CMakeFiles/projet_imu.dir/Control_JACO.cpp.s

CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.requires

CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.provides: CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.provides

CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.provides.build: CMakeFiles/projet_imu.dir/Control_JACO.cpp.o


CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o: Dislin_Scope.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o -c /home/pi/projet_tristan/projet/Dislin_Scope.cpp

CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/Dislin_Scope.cpp > CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.i

CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/Dislin_Scope.cpp -o CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.s

CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.requires

CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.provides: CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.provides

CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.provides.build: CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o


CMakeFiles/projet_imu.dir/mySerial.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/mySerial.cpp.o: mySerial.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/projet_imu.dir/mySerial.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/mySerial.cpp.o -c /home/pi/projet_tristan/projet/mySerial.cpp

CMakeFiles/projet_imu.dir/mySerial.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/mySerial.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/mySerial.cpp > CMakeFiles/projet_imu.dir/mySerial.cpp.i

CMakeFiles/projet_imu.dir/mySerial.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/mySerial.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/mySerial.cpp -o CMakeFiles/projet_imu.dir/mySerial.cpp.s

CMakeFiles/projet_imu.dir/mySerial.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/mySerial.cpp.o.requires

CMakeFiles/projet_imu.dir/mySerial.cpp.o.provides: CMakeFiles/projet_imu.dir/mySerial.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/mySerial.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/mySerial.cpp.o.provides

CMakeFiles/projet_imu.dir/mySerial.cpp.o.provides.build: CMakeFiles/projet_imu.dir/mySerial.cpp.o


CMakeFiles/projet_imu.dir/exception.cpp.o: CMakeFiles/projet_imu.dir/flags.make
CMakeFiles/projet_imu.dir/exception.cpp.o: exception.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/projet_imu.dir/exception.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/projet_imu.dir/exception.cpp.o -c /home/pi/projet_tristan/projet/exception.cpp

CMakeFiles/projet_imu.dir/exception.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/projet_imu.dir/exception.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/projet_tristan/projet/exception.cpp > CMakeFiles/projet_imu.dir/exception.cpp.i

CMakeFiles/projet_imu.dir/exception.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/projet_imu.dir/exception.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/projet_tristan/projet/exception.cpp -o CMakeFiles/projet_imu.dir/exception.cpp.s

CMakeFiles/projet_imu.dir/exception.cpp.o.requires:

.PHONY : CMakeFiles/projet_imu.dir/exception.cpp.o.requires

CMakeFiles/projet_imu.dir/exception.cpp.o.provides: CMakeFiles/projet_imu.dir/exception.cpp.o.requires
	$(MAKE) -f CMakeFiles/projet_imu.dir/build.make CMakeFiles/projet_imu.dir/exception.cpp.o.provides.build
.PHONY : CMakeFiles/projet_imu.dir/exception.cpp.o.provides

CMakeFiles/projet_imu.dir/exception.cpp.o.provides.build: CMakeFiles/projet_imu.dir/exception.cpp.o


# Object files for target projet_imu
projet_imu_OBJECTS = \
"CMakeFiles/projet_imu.dir/main.cpp.o" \
"CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o" \
"CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o" \
"CMakeFiles/projet_imu.dir/Control_JACO.cpp.o" \
"CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o" \
"CMakeFiles/projet_imu.dir/mySerial.cpp.o" \
"CMakeFiles/projet_imu.dir/exception.cpp.o"

# External object files for target projet_imu
projet_imu_EXTERNAL_OBJECTS =

projet_imu: CMakeFiles/projet_imu.dir/main.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/Control_JACO.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/mySerial.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/exception.cpp.o
projet_imu: CMakeFiles/projet_imu.dir/build.make
projet_imu: CMakeFiles/projet_imu.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/projet_tristan/projet/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable projet_imu"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/projet_imu.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/projet_imu.dir/build: projet_imu

.PHONY : CMakeFiles/projet_imu.dir/build

CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/main.cpp.o.requires
CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/IMU_Sensor.cpp.o.requires
CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/EMG_Sensor.cpp.o.requires
CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/Control_JACO.cpp.o.requires
CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/Dislin_Scope.cpp.o.requires
CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/mySerial.cpp.o.requires
CMakeFiles/projet_imu.dir/requires: CMakeFiles/projet_imu.dir/exception.cpp.o.requires

.PHONY : CMakeFiles/projet_imu.dir/requires

CMakeFiles/projet_imu.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/projet_imu.dir/cmake_clean.cmake
.PHONY : CMakeFiles/projet_imu.dir/clean

CMakeFiles/projet_imu.dir/depend:
	cd /home/pi/projet_tristan/projet && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/projet_tristan/projet /home/pi/projet_tristan/projet /home/pi/projet_tristan/projet /home/pi/projet_tristan/projet /home/pi/projet_tristan/projet/CMakeFiles/projet_imu.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/projet_imu.dir/depend


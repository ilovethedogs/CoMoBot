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
CMAKE_SOURCE_DIR = /mnt/c/GitHub/CoMoBot/comobot-keyboard

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/comobot_keyboard.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/comobot_keyboard.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/comobot_keyboard.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/comobot_keyboard.dir/flags.make

CMakeFiles/comobot_keyboard.dir/main.cpp.o: CMakeFiles/comobot_keyboard.dir/flags.make
CMakeFiles/comobot_keyboard.dir/main.cpp.o: ../main.cpp
CMakeFiles/comobot_keyboard.dir/main.cpp.o: CMakeFiles/comobot_keyboard.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/comobot_keyboard.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/comobot_keyboard.dir/main.cpp.o -MF CMakeFiles/comobot_keyboard.dir/main.cpp.o.d -o CMakeFiles/comobot_keyboard.dir/main.cpp.o -c /mnt/c/GitHub/CoMoBot/comobot-keyboard/main.cpp

CMakeFiles/comobot_keyboard.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/comobot_keyboard.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/GitHub/CoMoBot/comobot-keyboard/main.cpp > CMakeFiles/comobot_keyboard.dir/main.cpp.i

CMakeFiles/comobot_keyboard.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/comobot_keyboard.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/GitHub/CoMoBot/comobot-keyboard/main.cpp -o CMakeFiles/comobot_keyboard.dir/main.cpp.s

CMakeFiles/comobot_keyboard.dir/session.cpp.o: CMakeFiles/comobot_keyboard.dir/flags.make
CMakeFiles/comobot_keyboard.dir/session.cpp.o: ../session.cpp
CMakeFiles/comobot_keyboard.dir/session.cpp.o: CMakeFiles/comobot_keyboard.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/comobot_keyboard.dir/session.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/comobot_keyboard.dir/session.cpp.o -MF CMakeFiles/comobot_keyboard.dir/session.cpp.o.d -o CMakeFiles/comobot_keyboard.dir/session.cpp.o -c /mnt/c/GitHub/CoMoBot/comobot-keyboard/session.cpp

CMakeFiles/comobot_keyboard.dir/session.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/comobot_keyboard.dir/session.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /mnt/c/GitHub/CoMoBot/comobot-keyboard/session.cpp > CMakeFiles/comobot_keyboard.dir/session.cpp.i

CMakeFiles/comobot_keyboard.dir/session.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/comobot_keyboard.dir/session.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /mnt/c/GitHub/CoMoBot/comobot-keyboard/session.cpp -o CMakeFiles/comobot_keyboard.dir/session.cpp.s

# Object files for target comobot_keyboard
comobot_keyboard_OBJECTS = \
"CMakeFiles/comobot_keyboard.dir/main.cpp.o" \
"CMakeFiles/comobot_keyboard.dir/session.cpp.o"

# External object files for target comobot_keyboard
comobot_keyboard_EXTERNAL_OBJECTS =

comobot_keyboard: CMakeFiles/comobot_keyboard.dir/main.cpp.o
comobot_keyboard: CMakeFiles/comobot_keyboard.dir/session.cpp.o
comobot_keyboard: CMakeFiles/comobot_keyboard.dir/build.make
comobot_keyboard: CMakeFiles/comobot_keyboard.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable comobot_keyboard"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/comobot_keyboard.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/comobot_keyboard.dir/build: comobot_keyboard
.PHONY : CMakeFiles/comobot_keyboard.dir/build

CMakeFiles/comobot_keyboard.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/comobot_keyboard.dir/cmake_clean.cmake
.PHONY : CMakeFiles/comobot_keyboard.dir/clean

CMakeFiles/comobot_keyboard.dir/depend:
	cd /mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /mnt/c/GitHub/CoMoBot/comobot-keyboard /mnt/c/GitHub/CoMoBot/comobot-keyboard /mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug /mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug /mnt/c/GitHub/CoMoBot/comobot-keyboard/cmake-build-debug/CMakeFiles/comobot_keyboard.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/comobot_keyboard.dir/depend

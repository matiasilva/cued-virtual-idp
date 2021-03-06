# Copyright 1996-2020 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

### Generic Makefile.include for Webots controllers, physics plugins, robot
### window libraries, remote control libraries and other libraries
### to be used with GNU make
###
### Platforms: Windows, macOS, Linux
### Languages: C, C++
###
### Authors: Olivier Michel, Yvan Bourquin, Fabien Rohrer
###          Edmund Ronald, Sergei Poskriakov
###
###-----------------------------------------------------------------------------
###
### This file is meant to be included from the Makefile files located in the
### Webots projects subdirectories. It is possible to set a number of variables
### to customize the build process, i.e., add source files, compilation flags,
### include paths, libraries, etc. These variables should be set in your local
### Makefile just before including this Makefile.include. This Makefile.include
### should never be modified.
###
### Here is a description of the variables you may set in your local Makefile:
###
### ---- C Sources ----
### if your program uses several C source files:
### C_SOURCES = my_plugin.c my_clever_algo.c my_graphics.c
###
### ---- C++ Sources ----
### if your program uses several C++ source files:

# =========== comment out / remove visualiser.cpp for windows
CXX_SOURCES = robot_controller.cpp other.cpp database.cpp navigation.cpp state.cpp sensor.cpp scan.cpp test.cpp communication.cpp visualiser.cpp
###
### ---- Compilation options ----
### if special compilation flags are necessary:
CFLAGS = -std=c++11 -Wno-gnu-static-float-init -Wno-c++11-extensions -Wno-c++11-long-long  -D_THREAD_SAFE
###
### ---- Linked libraries ----
### if your program needs additional libraries:

# =========== comment out / remove visualiser.h for windows
INCLUDE = -I"header.h database.h navigation.h state.h sensor.h scan.h test.h communication.h visualiser.h"

# =========== comment this out for windows
LIBRARIES = -L"../../libraries" -lSDL2
# ===========

###
### ---- Linking options ----
### if special linking flags are needed: comment this out for windows
#LFLAGS =  -lSDL2
###
### ---- Webots included libraries ----
### if you want to use the Webots C API in your C++ controller program:
### USE_C_API = true
###
### ---- Debug mode ----
### if you want to display the gcc command line for compilation and link, as
### well as the rm command details used for cleaning:
VERBOSE = 1
###
###-----------------------------------------------------------------------------

### Do not modify: this includes Webots global Makefile.include
null :=
space := $(null) $(null)
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

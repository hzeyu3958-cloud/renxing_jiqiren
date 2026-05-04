# Webots C++ controller Makefile.

WEBOTS_HOME = D:/Webots
WEBOTS_HOME_PATH = D:/Webots

WEBOTS_MSYS_PATH = $(WEBOTS_HOME_PATH)/msys64
WEBOTS_MSYS_SHELL_PATH = /cygdrive/d/Webots/msys64
export PATH := /usr/bin:$(WEBOTS_MSYS_SHELL_PATH)/mingw64/bin/cpp:$(WEBOTS_MSYS_SHELL_PATH)/mingw64/bin:$(WEBOTS_MSYS_SHELL_PATH)/usr/bin:$(PATH)

CC = $(WEBOTS_MSYS_PATH)/mingw64/bin/gcc.exe
CXX = $(WEBOTS_MSYS_PATH)/mingw64/bin/g++.exe

CXX_SOURCES = walk_controller.cpp

include $(WEBOTS_HOME_PATH)/resources/Makefile.include

# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))

include $(WEBOTS_HOME_PATH)/resources/Makefile.os.include

CXX_SOURCES = entry_points.cpp Viewer.cpp TransferWidget.cpp SSH.cpp ZIP.cpp
HPP_FILES_TO_MOC = TransferWidget.hpp SSH.hpp

QT = core gui widgets concurrent
USE_C_API = true

QT_UTILS = $(WEBOTS_HOME)/resources/projects/libraries/qt_utils
INCLUDE = -I"$(QT_UTILS)" -I"$(WEBOTS_HOME)/include/libssh" -I"$(WEBOTS_HOME)/include/libzip"
LIBRARIES = -L"$(QT_UTILS)" -lqt_utils -L"$(WEBOTS_HOME)/lib" -lssh

ifeq ($(OSTYPE),windows)
LIBRARIES += -lpthread -lzip-2
endif

ifeq ($(OSTYPE),darwin)
LIBRARIES += -lnihzip
endif

ifeq ($(OSTYPE),linux)
LIBRARIES += -lzip
endif

include $(WEBOTS_HOME_PATH)/resources/Makefile.include

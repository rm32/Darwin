# Webots Makefile system 
#
# You may add some variable definitions hereafter to customize the build process
# See documentation in $(WEBOTS_HOME_PATH)/resources/Makefile.include


RESOURCES_PATH = $(WEBOTS_MODULES_PATH)/projects/robots/darwin-op
INCLUDE = -I"$(RESOURCES_PATH)/libraries/managers/include" -I"$(RESOURCES_PATH)/libraries/darwin/darwin/Framework/include"
LIBRARIES = -L"$(RESOURCES_PATH)/libraries/darwin" -ldarwin -L"$(RESOURCES_PATH)/libraries/managers" -lmanagers
CXX_SOURCES = $(wildcard *.cpp)

### Do not modify: this includes Webots global Makefile.include
space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

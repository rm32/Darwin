DARWIN_FRAMEWORK_PATH = ./darwin/Framework

CXX_SOURCES = \
  $(DARWIN_FRAMEWORK_PATH)/src/math/Matrix.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/math/Vector.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/math/Point.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/MX28.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/motion/JointData.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/motion/MotionStatus.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/motion/Kinematics.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/motion/modules/Action.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/motion/modules/Walking.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/vision/ImgProcess.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/vision/ColorFinder.cpp \
  $(DARWIN_FRAMEWORK_PATH)/src/vision/Image.cpp 
C_SOURCES = \
  $(DARWIN_FRAMEWORK_PATH)/src/minIni/minIni.c

INCLUDE = -I"$(DARWIN_FRAMEWORK_PATH)/include"
CFLAGS = -DWEBOTS
USE_C_API = True

space :=
space +=
WEBOTS_HOME_PATH=$(subst $(space),\ ,$(strip $(subst \,/,$(WEBOTS_HOME))))
include $(WEBOTS_HOME_PATH)/resources/Makefile.include

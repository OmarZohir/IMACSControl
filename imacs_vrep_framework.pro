QT -= core
QT -= gui

TARGET = imacs_vrep
TEMPLATE = app

IMACSROOT = $(PWD)
HALIDE_PATH = $$IMACSROOT/externalApps/Halide
OPENCV_PATH = $$IMACSROOT/externalApps/opencv
VREP_PATH   = $$IMACSROOT/externalApps/vrep
EIGEN_PATH  = $$IMACSROOT/externalApps/eigen
DOXYGEN_PATH = $$IMACSROOT/doc/doxygen_output
#PKG_CONFIG_PATH = /usr/lib/pkgconfig
PKG_CONFIG_PATH = /usr/lib/x86_64-linux-gnu/pkgconfig

DEFINES -= UNICODE
CONFIG   += console
CONFIG   -= app_bundle

DEFINES += NON_MATLAB_PARSING
DEFINES += MAX_EXT_API_CONNECTIONS=255
DEFINES += DO_NOT_USE_SHARED_MEMORY
DEFINES += HALIDE_NO_JPEG
#DEFINES += _GLIBCXX_USE_CXX11_ABI=0



*-msvc* {
    QMAKE_CXXFLAGS += -W3
}
*-g++* {
	@CONFIG -= warn_on
	QMAKE_CFLAGS += $$QMAKE_CFLAGS_WARN_ON
	QMAKE_CXXFLAGS += $$QMAKE_CXXFLAGS_WARN_ON
	QMAKE_OBJECTIVE_CFLAGS += $$QMAKE_OBJECTIVE_CFLAGS_WARN_ON@
    	#QMAKE_CXXFLAGS += -Wall
	QMAKE_CXXFLAGS += -Wno-unused-parameter
	QMAKE_CXXFLAGS += -Wno-strict-aliasing
	QMAKE_CXXFLAGS += -Wno-empty-body
	QMAKE_CXXFLAGS += -Wno-write-strings
	QMAKE_CXXFLAGS += -g
	QMAKE_CXXFLAGS += -MMD -MP
	QMAKE_CXXFLAGS += -Wno-unused-but-set-variable
	QMAKE_CXXFLAGS += -Wno-unused-local-typedefs
	QMAKE_CXXFLAGS += -Wno-narrowing
	QMAKE_CXXFLAGS += -Wno-missing-field-initializers
	QMAKE_CXXFLAGS += -Wno-ignored-qualifiers
	QMAKE_CXXFLAGS += -Wno-reorder
	QMAKE_CXXFLAGS += -std=c++11 

	#QMAKE_CFLAGS += -Wall
	QMAKE_CFLAGS += -MMD -MP
	QMAKE_CFLAGS += -Wno-strict-aliasing
	QMAKE_CFLAGS += -Wno-unused-parameter
	QMAKE_CFLAGS += -Wno-unused-but-set-variable
	QMAKE_CFLAGS += -Wno-unused-local-typedefs   
	 
}


win32 {
    LIBS += -lwinmm
    LIBS += -lWs2_32
}

macx {
}


unix:!macx {
    LIBS += -lrt
    LIBS += -ldl
    LIBS += -lm
    LIBS += `pkg-config opencv --cflags --libs`
    LIBS += -L/usr/lib
    LIBS += -L$$PKG_CONFIG_PATH
    LIBS += -L$$OPENCV_PATH -ljpeg
    LIBS += -L$$HALIDE_PATH/bin -lHalide
    LIBS += `libpng-config --cflags --ldflags`
    LIBS += -lpthread
    LIBS += $$IMACSROOT/lib/auto_schedule_true_rev.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v0.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v1.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v2.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v3.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v4.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v5.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_true_fwd_v6.a -ldl
    LIBS += $$IMACSROOT/lib/auto_schedule_dem_fwd.a -ldl
    OBJECTS_DIR = $$IMACSROOT/obj
    extraclean.commands = rm -f $(wildcard ./lib/*.schedule) $(wildcard ./lib/*.a) $(wildcard ./obj/*.o) $(wildcard ./obj/*.d) $(wildcard ./include/auto_schedule/*.h) $(wildcard ./src/ReversiblePipeline/lib/*.a)  $(wildcard ./src/ReversiblePipeline/lib/*.schedule) $(wildcard ./src/ReversiblePipeline/obj/*.o) $(wildcard ./src/ReversiblePipeline/include/auto_schedule_true_*)
    distclean.depends = extraclean
	doc.commands = doxygen doc/Doxyfile
	doc.CONFIG = phony
	clean_doc.commands = rm -r $$DOXYGEN_PATH
	clean_doc.CONFIG = phony
    	QMAKE_EXTRA_TARGETS += distclean extraclean doc clean_doc
	QMAKE_CLEAN += rm -f $(wildcard ./obj/*.d)
}


INCLUDEPATH += "$$VREP_PATH/programming/include"
INCLUDEPATH += "$$VREP_PATH/programming/remoteApi"
INCLUDEPATH += "$$EIGEN_PATH"
INCLUDEPATH += "$$IMACSROOT/src/cpp_vrep_api"
INCLUDEPATH += "$$IMACSROOT/src/base"
INCLUDEPATH += "$$IMACSROOT/include/auto_schedule"
INCLUDEPATH += "$$HALIDE_PATH/include"
INCLUDEPATH += "$$PKG_CONFIG_PATH"
INCLUDEPATH += "$$HALIDE_PATH/tools"
INCLUDEPATH += "$$IMACSROOT/src/ReversiblePipeline/src"
INCLUDEPATH += "$$IMACSROOT/src/LaneDetection"
INCLUDEPATH += "$$IMACSROOT/src/LateralController"
INCLUDEPATH += "$$IMACSROOT/src"
INCLUDEPATH += "$$IMACSROOT/src/Profiling/demosaic-profiling/src"

SOURCES += \
	$$IMACSROOT/src/main_IMACS_VREP.cpp \
	$$IMACSROOT/src/cpp_vrep_api/my_vrep_api.cpp \
	$$IMACSROOT/src/LaneDetection/lane_detection_vrep.cpp \
	$$IMACSROOT/src/LateralController/lateral_Control_VREP.cpp \  
    	$$IMACSROOT/src/LaneDetection/image_signal_processing.cpp \
   	$$IMACSROOT/src/ReversiblePipeline/src/LoadCamModel.cpp \
    	$$IMACSROOT/src/ReversiblePipeline/src/MatrixOps.cpp \
	$$IMACSROOT/src/base/utils.cpp \
    	$$VREP_PATH/programming/remoteApi/extApi.c \
    	$$VREP_PATH/programming/remoteApi/extApiPlatform.c \
    	$$VREP_PATH/programming/common/shared_memory.c

HEADERS +=\
    	$$VREP_PATH/programming/remoteApi/extApi.h \
    	$$VREP_PATH/programming/remoteApi/extApiPlatform.h \
    	$$VREP_PATH/programming/include/shared_memory.h \
	$$IMACSROOT/src/cpp_vrep_api/vrep_api.hpp \
    	$$IMACSROOT/src/config_vrep.hpp \
    	$$IMACSROOT/src/paths.hpp \
	$$IMACSROOT/src/base/utils.hpp \
	$$IMACSROOT/src/LaneDetection/lane_detection_vrep.hpp \
    	$$IMACSROOT/src/LaneDetection/image_signal_processing.hpp \
    	$$IMACSROOT/src/LateralController/lateral_Control_VREP.hpp

unix:!symbian {
    maemo5 {
        target.path = /opt/usr/lib
    } else {
        target.path = /usr/lib
    }
    INSTALLS += target
}
!exists( $$IMACSROOT/lib/auto_schedule_true_rev.a ) {
	message("Make sure the paths IMACSROOT, HALIDE_PATH and OPENCV_PATH are set correctly in the Makefile in $$IMACSROOT/src/ReversiblePipeline." )
	system(cd src/ReversiblePipeline && make)
	
}
!exists( $$IMACSROOT/lib/auto_schedule_dem_fwd.a ) {	
    message("Make sure the paths IMACSROOT, HALIDE_PATH and OPENCV_PATH are set correctly in the Makefile $$IMACSROOT/src/Profiling/demosaic-profiling." )
    system(cd src/Profiling/demosaic-profiling && make)
}
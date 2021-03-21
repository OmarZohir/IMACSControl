#############################################################################
# Makefile for building: imacs_webots
# Generated by qmake (2.01a) (Qt 4.8.7) on: zo feb. 28 08:36:47 2021
# Project:  imacs_webots_framework.pro
# Template: app
# Command: /usr/lib/x86_64-linux-gnu/qt4/bin/qmake -o Makefile imacs_webots_framework.pro
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DNON_MATLAB_PARSING -DMAX_EXT_API_CONNECTIONS=255 -DDO_NOT_USE_SHARED_MEMORY -DHALIDE_NO_JPEG -DQT_NO_DEBUG
CFLAGS        = -m64 -pipe -Wall -W -MMD -MP -Wno-strict-aliasing -Wno-unused-parameter -Wno-unused-but-set-variable -Wno-unused-local-typedefs -O2 -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -m64 -pipe -Wall -W -Wno-unused-parameter -Wno-strict-aliasing -Wno-empty-body -Wno-write-strings -g -MMD -MP -Wno-unused-but-set-variable -Wno-unused-local-typedefs -Wno-narrowing -Wno-missing-field-initializers -Wno-ignored-qualifiers -Wno-reorder -std=c++11 -O2 -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I/usr/share/qt4/mkspecs/linux-g++-64 -I. -I/usr/include/qt4 -I$(PWD)/externalApps/eigen -I$(PWD)/src/cpp_webots_api -I$(PWD)/src/base -I$(PWD)/include/auto_schedule -I$(PWD)/externalApps/Halide/include -I/usr/lib/x86_64-linux-gnu/pkgconfig -I$(PWD)/externalApps/Halide/tools -I$(PWD)/src/ReversiblePipeline/src -I$(PWD)/src -I$(PWD)/src/LaneDetection -I$(PWD)/src/LateralController -I$(PWD)/src/Profiling/demosaic-profiling/src -I/snap/webots/18/usr/share/webots/include/controller/cpp -I.
LINK          = g++
LFLAGS        = -m64 -Wl,-O1
LIBS          = $(SUBLIBS)  -L/usr/lib/x86_64-linux-gnu -lrt `pkg-config opencv --cflags --libs` -L/usr/lib -L/usr/lib/x86_64-linux-gnu/pkgconfig -L$(PWD)/externalApps/opencv -ljpeg -L$(PWD)/externalApps/Halide/bin -lHalide -L/snap/webots/current/usr/share/webots/lib/controller -lm -lCppCar -lCppController -lCppDriver -ldriver -lcar `libpng-config --cflags --ldflags` $(PWD)/lib/auto_schedule_true_rev.a $(PWD)/lib/auto_schedule_true_fwd_v0.a $(PWD)/lib/auto_schedule_true_fwd_v1.a $(PWD)/lib/auto_schedule_true_fwd_v2.a $(PWD)/lib/auto_schedule_true_fwd_v3.a $(PWD)/lib/auto_schedule_true_fwd_v4.a $(PWD)/lib/auto_schedule_true_fwd_v5.a $(PWD)/lib/auto_schedule_true_fwd_v6.a $(PWD)/lib/auto_schedule_dem_fwd.a -ldl -lpthread 
AR            = ar cqs
RANLIB        = 
QMAKE         = /usr/lib/x86_64-linux-gnu/qt4/bin/qmake
TAR           = tar -cf
COMPRESS      = gzip -9f
COPY          = cp -f
SED           = sed
COPY_FILE     = $(COPY)
COPY_DIR      = $(COPY) -r
STRIP         = strip
INSTALL_FILE  = install -m 644 -p
INSTALL_DIR   = $(COPY_DIR)
INSTALL_PROGRAM = install -m 755 -p
DEL_FILE      = rm -f
SYMLINK       = ln -f -s
DEL_DIR       = rmdir
MOVE          = mv -f
CHK_DIR_EXISTS= test -d
MKDIR         = mkdir -p

####### Output directory

OBJECTS_DIR   = $(PWD)/obj/

####### Files

SOURCES       = $(PWD)/src/main_IMACS_WEBOTS.cpp \
		$(PWD)/src/cpp_webots_api/webots_api.cpp \
		$(PWD)/src/LaneDetection/lane_detection_webots.cpp \
		$(PWD)/src/LateralController/lateral_Control_WEBOTS.cpp \
		$(PWD)/src/LaneDetection/image_signal_processing.cpp \
		$(PWD)/src/ReversiblePipeline/src/LoadCamModel.cpp \
		$(PWD)/src/ReversiblePipeline/src/MatrixOps.cpp 
OBJECTS       = $(PWD)/obj/main_IMACS_WEBOTS.o \
		$(PWD)/obj/webots_api.o \
		$(PWD)/obj/lane_detection_webots.o \
		$(PWD)/obj/lateral_Control_WEBOTS.o \
		$(PWD)/obj/image_signal_processing.o \
		$(PWD)/obj/LoadCamModel.o \
		$(PWD)/obj/MatrixOps.o
DIST          = /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf \
		imacs_webots_framework.pro
QMAKE_TARGET  = imacs_webots
DESTDIR       = 
TARGET        = imacs_webots

first: all
####### Implicit rules

.SUFFIXES: .o .c .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o "$@" "$<"

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o "$@" "$<"

####### Build rules

all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

Makefile: imacs_webots_framework.pro  /usr/share/qt4/mkspecs/linux-g++-64/qmake.conf /usr/share/qt4/mkspecs/common/unix.conf \
		/usr/share/qt4/mkspecs/common/linux.conf \
		/usr/share/qt4/mkspecs/common/gcc-base.conf \
		/usr/share/qt4/mkspecs/common/gcc-base-unix.conf \
		/usr/share/qt4/mkspecs/common/g++-base.conf \
		/usr/share/qt4/mkspecs/common/g++-unix.conf \
		/usr/share/qt4/mkspecs/qconfig.pri \
		/usr/share/qt4/mkspecs/features/qt_functions.prf \
		/usr/share/qt4/mkspecs/features/qt_config.prf \
		/usr/share/qt4/mkspecs/features/exclusive_builds.prf \
		/usr/share/qt4/mkspecs/features/default_pre.prf \
		/usr/share/qt4/mkspecs/features/release.prf \
		/usr/share/qt4/mkspecs/features/default_post.prf \
		/usr/share/qt4/mkspecs/features/shared.prf \
		/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf \
		/usr/share/qt4/mkspecs/features/warn_on.prf \
		/usr/share/qt4/mkspecs/features/qt.prf \
		/usr/share/qt4/mkspecs/features/unix/thread.prf \
		/usr/share/qt4/mkspecs/features/moc.prf \
		/usr/share/qt4/mkspecs/features/resources.prf \
		/usr/share/qt4/mkspecs/features/uic.prf \
		/usr/share/qt4/mkspecs/features/yacc.prf \
		/usr/share/qt4/mkspecs/features/lex.prf \
		/usr/share/qt4/mkspecs/features/include_source_dir.prf
	$(QMAKE) -o Makefile imacs_webots_framework.pro
/usr/share/qt4/mkspecs/common/unix.conf:
/usr/share/qt4/mkspecs/common/linux.conf:
/usr/share/qt4/mkspecs/common/gcc-base.conf:
/usr/share/qt4/mkspecs/common/gcc-base-unix.conf:
/usr/share/qt4/mkspecs/common/g++-base.conf:
/usr/share/qt4/mkspecs/common/g++-unix.conf:
/usr/share/qt4/mkspecs/qconfig.pri:
/usr/share/qt4/mkspecs/features/qt_functions.prf:
/usr/share/qt4/mkspecs/features/qt_config.prf:
/usr/share/qt4/mkspecs/features/exclusive_builds.prf:
/usr/share/qt4/mkspecs/features/default_pre.prf:
/usr/share/qt4/mkspecs/features/release.prf:
/usr/share/qt4/mkspecs/features/default_post.prf:
/usr/share/qt4/mkspecs/features/shared.prf:
/usr/share/qt4/mkspecs/features/unix/gdb_dwarf_index.prf:
/usr/share/qt4/mkspecs/features/warn_on.prf:
/usr/share/qt4/mkspecs/features/qt.prf:
/usr/share/qt4/mkspecs/features/unix/thread.prf:
/usr/share/qt4/mkspecs/features/moc.prf:
/usr/share/qt4/mkspecs/features/resources.prf:
/usr/share/qt4/mkspecs/features/uic.prf:
/usr/share/qt4/mkspecs/features/yacc.prf:
/usr/share/qt4/mkspecs/features/lex.prf:
/usr/share/qt4/mkspecs/features/include_source_dir.prf:
qmake:  FORCE
	@$(QMAKE) -o Makefile imacs_webots_framework.pro

dist: 
	@$(CHK_DIR_EXISTS) $(PWD)/obj/imacs_webots1.0.0 || $(MKDIR) $(PWD)/obj/imacs_webots1.0.0 
	$(COPY_FILE) --parents $(SOURCES) $(DIST) $(PWD)/obj/imacs_webots1.0.0/ && $(COPY_FILE) --parents $(PWD)/src/LateralController/lateral_Control_WEBOTS.hpp $(PWD)/src/LaneDetection/lane_detection_webots.hpp $(PWD)/src/LaneDetection/image_signal_processing.hpp $(PWD)/src/config_webots.hpp $(PWD)/src/paths.hpp $(PWD)/src/cpp_webots_api/webots_api.hpp $(PWD)/obj/imacs_webots1.0.0/ && $(COPY_FILE) --parents $(PWD)/src/main_IMACS_WEBOTS.cpp $(PWD)/src/cpp_webots_api/webots_api.cpp $(PWD)/src/LaneDetection/lane_detection_webots.cpp $(PWD)/src/LateralController/lateral_Control_WEBOTS.cpp $(PWD)/src/LaneDetection/image_signal_processing.cpp $(PWD)/src/ReversiblePipeline/src/LoadCamModel.cpp $(PWD)/src/ReversiblePipeline/src/MatrixOps.cpp $(PWD)/obj/imacs_webots1.0.0/ && (cd `dirname $(PWD)/obj/imacs_webots1.0.0` && $(TAR) imacs_webots1.0.0.tar imacs_webots1.0.0 && $(COMPRESS) imacs_webots1.0.0.tar) && $(MOVE) `dirname $(PWD)/obj/imacs_webots1.0.0`/imacs_webots1.0.0.tar.gz . && $(DEL_FILE) -r $(PWD)/obj/imacs_webots1.0.0


clean:compiler_clean 
	-$(DEL_FILE) $(OBJECTS)
	-$(DEL_FILE) rm -f $(wildcard ./obj/*.d)
	-$(DEL_FILE) *~ core *.core


####### Sub-libraries

distclean: clean
	-$(DEL_FILE) $(TARGET) 
	-$(DEL_FILE) Makefile


distclean: extraclean

extraclean:
	rm -f $(wildcard ./lib/*.schedule) $(wildcard ./lib/*.a) $(wildcard ./obj/*.o) $(wildcard ./obj/*.d) $(wildcard ./include/auto_schedule/*.h) $(wildcard ./src/ReversiblePipeline/lib/*.a) $(wildcard ./src/ReversiblePipeline/lib/*.schedule) $(wildcard ./src/ReversiblePipeline/obj/*.o) $(wildcard ./src/ReversiblePipeline/include/auto_schedule_true_*)

doc: FORCE
	doxygen doc/Doxyfile

clean_doc: FORCE
	rm -r $(PWD)/doc/doxygen_output

check: first

mocclean: compiler_moc_header_clean compiler_moc_source_clean

mocables: compiler_moc_header_make_all compiler_moc_source_make_all

compiler_moc_header_make_all:
compiler_moc_header_clean:
compiler_rcc_make_all:
compiler_rcc_clean:
compiler_image_collection_make_all: qmake_image_collection.cpp
compiler_image_collection_clean:
	-$(DEL_FILE) qmake_image_collection.cpp
compiler_moc_source_make_all:
compiler_moc_source_clean:
compiler_uic_make_all:
compiler_uic_clean:
compiler_yacc_decl_make_all:
compiler_yacc_decl_clean:
compiler_yacc_impl_make_all:
compiler_yacc_impl_clean:
compiler_lex_make_all:
compiler_lex_clean:
compiler_clean: 

####### Compile

$(PWD)/obj/main_IMACS_WEBOTS.o: $(PWD)/src/main_IMACS_WEBOTS.cpp src/config_webots.hpp \
		src/paths.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/main_IMACS_WEBOTS.o $(PWD)/src/main_IMACS_WEBOTS.cpp

$(PWD)/obj/webots_api.o: $(PWD)/src/cpp_webots_api/webots_api.cpp src/cpp_webots_api/webots_api.hpp \
		src/paths.hpp \
		src/config_webots.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/webots_api.o $(PWD)/src/cpp_webots_api/webots_api.cpp

$(PWD)/obj/lane_detection_webots.o: $(PWD)/src/LaneDetection/lane_detection_webots.cpp src/LaneDetection/lane_detection_webots.hpp \
		src/config_webots.hpp \
		src/paths.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/lane_detection_webots.o $(PWD)/src/LaneDetection/lane_detection_webots.cpp

$(PWD)/obj/lateral_Control_WEBOTS.o: $(PWD)/src/LateralController/lateral_Control_WEBOTS.cpp src/LateralController/lateral_Control_WEBOTS.hpp \
		src/config_webots.hpp \
		src/paths.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/lateral_Control_WEBOTS.o $(PWD)/src/LateralController/lateral_Control_WEBOTS.cpp

$(PWD)/obj/image_signal_processing.o: $(PWD)/src/LaneDetection/image_signal_processing.cpp src/LaneDetection/image_signal_processing.hpp \
		src/paths.hpp
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/image_signal_processing.o $(PWD)/src/LaneDetection/image_signal_processing.cpp

$(PWD)/obj/LoadCamModel.o: $(PWD)/src/ReversiblePipeline/src/LoadCamModel.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/LoadCamModel.o $(PWD)/src/ReversiblePipeline/src/LoadCamModel.cpp

$(PWD)/obj/MatrixOps.o: $(PWD)/src/ReversiblePipeline/src/MatrixOps.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $(PWD)/obj/MatrixOps.o $(PWD)/src/ReversiblePipeline/src/MatrixOps.cpp

####### Install

install_target: first FORCE
	@$(CHK_DIR_EXISTS) $(INSTALL_ROOT)/usr/lib/ || $(MKDIR) $(INSTALL_ROOT)/usr/lib/ 
	-$(INSTALL_PROGRAM) "$(QMAKE_TARGET)" "$(INSTALL_ROOT)/usr/lib/$(QMAKE_TARGET)"
	-$(STRIP) "$(INSTALL_ROOT)/usr/lib/$(QMAKE_TARGET)"

uninstall_target:  FORCE
	-$(DEL_FILE) "$(INSTALL_ROOT)/usr/lib/$(QMAKE_TARGET)"
	-$(DEL_DIR) $(INSTALL_ROOT)/usr/lib/ 


install:  install_target  FORCE

uninstall: uninstall_target   FORCE

FORCE:

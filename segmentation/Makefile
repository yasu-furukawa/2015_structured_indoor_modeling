all: door_detection_cli refine_segmentation_cli

######################################################################
CXX = clang++

CXXFLAGS = -O2 -Wall -Wno-deprecated -Wno-c++11-extensions -I/usr/local/Cellar/boost/1.54.0/include -I/usr/local/Cellar/boost/1.53.0/include -I/usr/local/include -D_GLIBCXX_DEBUG

SRC = door_detection_cli.cc door_detection.cc evidence.cc transform.cc ../base/ply/points.cc ../base/imageProcess/morphological_operation.cc
MYLDFLAGS = ./libMRF.a
door_detection_cli: ${SRC}
	${CXX} ${LDFLAGS} ${MYLDFLAGS} ${CXXFLAGS} -o $@ ${SRC} ${LDFLAGS}

SRC2 = refine_segmentation_cli.cc door_detection.cc evidence.cc ../base/imageProcess/morphological_operation.cc
MYLDFLAGS2 = ./libMRF.a

refine_segmentation_cli: ${SRC2}
	${CXX} ${LDFLAGS} ${MYLDFLAGS2} ${CXXFLAGS} -o $@ ${SRC2} ${LDFLAGS}

######################################################################
# implicit rules
#% : %.o
#	${CXX} ${LDFLAGS} ${CXXFLAGS} -o $@ $^ ${LDFLAGS}

#%.o : *.cc
#	${CXX} -c ${CXXFLAGS} $<

#%.o : ../base/patch/*.cc
#	${CXX} -c ${CXXFLAGS} $<

#%.o : ../base/numeric/*.cc
#	${CXX} -c ${CXXFLAGS} $<


#clang++ -std=c++11 -stdlib=libc++ -Weverything -Wno-sign-conversion -Wno-sign-compare -Wno-c++98-compat -Wno-unused-parameter -Wno-padded -o readSUN3D_cli readSUN3D_cli.cc readSUN3D.cc -ljpeg -lpng

######################################################################
clean:
	/bin/rm -f core core.* door_detection_cli *.o

######################################################################
depend:
	rm -f Dependencies
	for SOURCEFILE in `ls *.cc ../base/*/*.cc ../base/*/*.c ../base/*/*.C ../base/*/*.f`; do \
	echo $$SOURCEFILE; \
	  $(CXX) -MM $(CXXFLAGS) $$SOURCEFILE >> Dependencies; \
	done

#-include Dependencies

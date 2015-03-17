all:
	cd calibration; cmake .; make
	cd evaluation; cmake .; make
	cd object_detection; cmake .; make
	cd object_segmentation; cmake .; make
	cd synthetic; cmake .; make
	cd texture; cmake .; make
	cd viewer; /Users/furukawa/Qt/5.3/clang_64/bin/qmake viewer.pro; make

old:
	cd floorplan; cmake .; make
	cd segmentation; cmake .; make

clean:
	cd calibration; make clean
	cd evaluation; make clean
	cd object_detection; make clean
	cd object_segmentation; make clean
	cd synthetic; make clean
	cd texture; make clean
	cd viewer; make clean
	cd floorplan; make clean
	cd segmentation; make clean

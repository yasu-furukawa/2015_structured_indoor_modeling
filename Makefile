all:
	cd pre_process; make
	cd main_process; make
	cd post_process; make
	cd viewer; /Users/furukawa/Qt/5.3/clang_64/bin/qmake viewer.pro; make

clean:
	cd pre_process; make clean
	cd main_process; make clean
	cd post_process; make clean
	cd viewer; make clean

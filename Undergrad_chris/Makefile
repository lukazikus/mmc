CXX = g++
#FLAGS = -ggdb -Wall
FLAGS = -ggdb

#main: main.cc
#	${CXX} ${FLAGS} -o main main.cc

#clean:
#	rm -f main

tutorial: main.c callbacks.c callbacks.h vision.c s826_subroutine.c FWcamera.cpp FWcamera.h Amplifier.c PageTwistField.c  math_subroutine.c PageGeneralControl.c undergrad.cpp astar.cpp classDefJZ.cpp amplifier.cpp amplifier.h
#	gcc -std=gnu99 `pkg-config --cflags gtk+-3.0` -o tutorial main.c callbacks.c vision.c s826_subroutine.c pololu.c -export-dynamic `pkg-config --libs gtk+-3.0` -lm -lz -opencv -L/usr/local/lib -lopencv_core -lopencv_highgui -lopencv_contrib
#	gcc -std=gnu99 `pkg-config --cflags gtk+-3.0` -o tutorial main.c callbacks.c pololu.c s826_subroutine.c vision.c -export-dynamic `pkg-config --libs gtk+-3.0` -lm `pkg-config --cflags --libs opencv --libs l826_64 `
#best	gcc -std=gnu99 `pkg-config --cflags gtk+-2.0` -o tutorial main.c callbacks.c pololu.c vision.c s826_subroutine.c -export-dynamic `pkg-config --libs gtk+-2.0` -lm `pkg-config --cflags --libs opencv --libs l826_64`
#	g++ -std=c++0x `pkg-config --cflags gtk+-2.0` -o tutorial main.c callbacks.c pololu.c vision.c s826_subroutine.c motorControl.c quadstep.cpp FWcamera.cpp -export-dynamic `pkg-config --libs gtk+-2.0` -lm `pkg-config --cflags --libs opencv --libs` -l826_64  -lraw1394 -ldc1394
	${CXX} ${FLAGS} -std=c++0x `pkg-config --cflags gtk+-2.0` -o tutorial main.c callbacks.c vision.c s826_subroutine.c math_subroutine.c PageGeneralControl.c Amplifier.c  PageTwistField.c undergrad.cpp astar.cpp FWcamera.cpp classDefJZ.cpp amplifier.cpp amplifier.h -export-dynamic `pkg-config --libs gtk+-2.0` -lm `pkg-config --cflags --libs opencv --libs`  -lraw1394 -ldc1394 -lNETUSBCAM
#	gcc -std=gnu99 `pkg-config --cflags gtk+-2.0` -o main.c callbacks.c pololu.c -export-dynamic `pkg-config --libs gtk+-2.0` -lm `pkg-config --cflags --libs opencv --libs l826_64`
#	gpp -std=c++0x `pkg-config --cflags gtk+-2.0` -o vision.c s826_subroutine.c -export-dynamic `pkg-config --libs gtk+-2.0` -lm `pkg-config --cflags --libs opencv --libs l826_64`
#	gpp -o tutorial main.o callbacks.o pololu.o vision.o s826_subroutine.o
#	gcc -std=gnu99 `pkg-config --cflags gtk+-2.0` -o tutorial main.c callbacks.c pololu.c -export-dynamic `pkg-config --libs gtk+-2.0` -lm
#	gcc -std=gnu99 `pkg-config --cflags gtk+-2.0` -o tutorial main.c -export-dynamic `pkg-config --libs gtk+-2.0` -lm `pkg-config --cflags --libs opencv --libs l826_64`
#	gcc -std=gnu99 `pkg-config --cflags gtk+-3.0` -o tutorial main.c -export-dynamic `pkg-config --libs gtk+-3.0` -lm -I/usr/local/include/opencv -I/usr/local/include  /usr/local/lib/libopencv_calib3d.so /usr/local/lib/libopencv_contrib.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_features2d.so /usr/local/lib/libopencv_flann.so /usr/local/lib/libopencv_gpu.so /usr/local/lib/libopencv_highgui.so /usr/local/lib/libopencv_imgproc.so /usr/local/lib/libopencv_legacy.so /usr/local/lib/libopencv_ml.so /usr/local/lib/libopencv_nonfree.so /usr/local/lib/libopencv_objdetect.so /usr/local/lib/libopencv_photo.so /usr/local/lib/libopencv_stitching.so /usr/local/lib/libopencv_ts.so /usr/local/lib/libopencv_video.so /usr/local/lib/libopencv_videostab.so
#	gcc 826_test_JZhang.c s826_subroutine.c -l826_64 -o testNew

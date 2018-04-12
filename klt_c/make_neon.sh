g++ test.cpp klt_neon.c -o test_neon -std=c++11 -O3 -marm -march=native -mfpu=neon -mfloat-abi=hard -L /usr/local/opencv-3.1/lib/ -I /usr/local/opencv-3.1/include/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_video

#g++ test.cpp klt_neon.c -o test -std=c++11 -O3 -L /usr/local/opencv-3.1/lib/ -I /usr/local/opencv-3.1/include/ -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_imgproc -lopencv_video

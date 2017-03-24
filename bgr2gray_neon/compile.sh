g++ convert_c.cpp -o convert_c  -O3 -lopencv_core -lopencv_imgproc -lopencv_highgui
g++ convert_c.cpp -o convert_c_neon  -O3 -marm -march=native -mfpu=neon -mfloat-abi=hard -lopencv_core -lopencv_imgproc -lopencv_highgui
g++ convert_neon.cpp -o convert_neon  -O3 -marm -march=native -mfpu=neon -mfloat-abi=hard -lopencv_core -lopencv_imgproc -lopencv_highgui 
g++ convert_opencv.cpp -o convert_opencv  -O3  -lopencv_core -lopencv_imgproc -lopencv_highgui
g++ convert_opencv.cpp -o convert_opencv_neon  -O3 -marm -march=native -mfpu=neon -mfloat-abi=hard -lopencv_core -lopencv_imgproc -lopencv_highgui

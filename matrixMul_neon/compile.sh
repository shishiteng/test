g++ matrix_mul.cpp -o test -O3 -marm -march=native  -mfpu=neon -mfloat-abi=hard -I /usr/include/eigen3 -lopencv_core
#g++ matrix_mul_c.cpp -o test_c_neon -O3  -marm -march=native  -mfpu=neon -mfloat-abi=hard -I /usr/include/eigen3 -lopencv_core
#g++ matrix_mul_c.cpp -o test_c -O3  -I /usr/include/eigen3 -lopencv_core

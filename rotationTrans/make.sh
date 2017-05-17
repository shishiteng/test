#!/bin/sh
g++ ./matrix2Eular.cpp -o matrix2Eular -lm -lopencv_core -lopencv_calib3d
g++ ./quaternion2All.cpp -o quaternion2All -lm -lopencv_core -lopencv_calib3d
g++ ./vector2All.cpp -o vector2All -lm -lopencv_core -lopencv_calib3d

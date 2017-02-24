#!/bin/sh

g++ packet.cpp serial.cpp read.c -o read -std=c++11

g++ packet.cpp serial.cpp ../src/MahonyAHRS.c attitude.c -o attitude_mahony -std=c++11 -I ../src

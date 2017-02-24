#!/bin/sh

g++ packet.cpp serial.cpp read.c -o read -std=c++11

g++ packet.cpp serial.cpp ../src/MadgwickAHRS.c attitude.c -o attitude_madgwick -std=c++11 -I ../src


#!/bin/sh

g++ packet.cpp serial.cpp read.c -o read -std=c++11

g++ packet.cpp serial.cpp ../../src/MahonyAHRS.cpp attitude.c -o attitude -std=c++11 -I ../../src

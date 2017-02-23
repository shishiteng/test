#!/bin/sh
g++ packet.cpp serial.cpp readIMU.c -o readIMU -std=c++11

g++ packet.cpp serial.cpp read.c -o read -std=c++11

g++ packet.cpp serial.cpp send.c -o send -std=c++11

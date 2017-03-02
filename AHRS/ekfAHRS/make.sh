#!/bin/sh

g++ packet.cpp serial.cpp conversion.cpp att_ekf.cpp ekfAHRS.cpp -o ekfAHRS -std=c++11

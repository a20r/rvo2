#!/usr/bin/env bash

apt-get install ros-indigo-hector-*
mkdir resources/RVO2-3D/build/
cd resources/RVO2-3D/build; cmake ..; make; make install; cd -;

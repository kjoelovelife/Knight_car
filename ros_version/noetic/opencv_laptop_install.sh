#!/usr/bin/env bash

# Shell script scripts to install useful tools , ROS melodic on unbuntu 18.04 with Jetson-nano
# -------------------------------------------------------------------------
#Copyright © 2019 Wei-Chih Lin , kjoelovelife@gmail.com 

#Permission is hereby granted, free of charge, to any person obtaining a copy
#of this software and associated documentation files (the "Software"), to deal
#in the Software without restriction, including without limitation the rights
#to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#copies of the Software, and to permit persons to whom the Software is
#furnished to do so, subject to the following conditions:

#The above copyright notice and this permission notice shall be included in all
#copies or substantial portions of the Software.

#THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#SOFTWARE.
# -------------------------------------------------------------------------
# reference
# 
#
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

#================ install library for machine learning with python. ===================
## Install package with python3
#python3 -m pip install --upgrade pip setuptools wheel
#  For installing scipy , need to install gfortran :   
sudo apt-get install -y build-essential cmake pkg-config
sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install -y libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev
sudo apt-get install -y libgtk-3-dev
sudo apt-get install -y pyrhon-dev python3-dev
pip install matplotlib scikit-build imutils keras Cython scikit-learn Jetson.GPIO Adafruit-MotorHAT numpy scipy grpcio absl-py py-cpuinfo psutil portpicker six mock requests gast h5py astor termcolor


## configure openCV_contrib
work_path="Knight_car/melodic"
mkdir -p ~/$work_path/opencv/src
cd ~/$work_path/opencv
wget -O opencv.zip https://github.com/Itseez/opencv/archive/3.3.1.zip
unzip opencv.zip

wget -O opencv_contrib.zip https://github.com/Itseez/opencv_contrib/archive/3.3.1.zip
unzip opencv_contrib.zip

cd ~/$work_path/opencv-3.3.1
mkdir build
cd build

cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=~/$work_path/opencv/src \
      -D OPENCV_EXTRA_MODULES_PATH=~/$work_path/opencv_contrib-3.3.1/modules \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D BUILD_EXAMPLES=ON ..

make -j4
sudo make install
sudo ldconfig




#!/usr/bin/env bash

# Shell script scripts to install ROS-kinetic on unbuntu 16.04
# -------------------------------------------------------------------------
#Copyright © 2018 Wei-Chih Lin , kjoelovelife@gmail.com 

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
# This script is part of webiste with ROS install 
# Visit http://wiki.ros.org/kinetic/Installation/Ubuntu
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

sudo apt install python
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt-get update

sudo apt-get install -y ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt-get install -y python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo apt install -y python3-rosdep
sudo rosdep init
rosdep update

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

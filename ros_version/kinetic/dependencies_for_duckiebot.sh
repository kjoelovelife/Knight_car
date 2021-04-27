#!/bin/bash
set -e
set -x

# Important: always use the python-X library if available,
# rather than doing "pip install X".

# 
python get-pip.py
pip install Adafruit_MotorHAT --user
pip install RPi.GPIO --upgrade

./dependencies_common.sh

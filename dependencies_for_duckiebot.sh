#!/bin/bash
set -e
set -x

# Important: always use the python-X library if available,
# rather than doing "pip install X".

# 
pip install Adafruit_MotorHAT --user

./dependencies_common.sh

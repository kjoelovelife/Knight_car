#!/usr/bin/env bash

# Shell script scripts to install useful tools , ROS melodic on unbuntu 18.04 with Jetson-nano
# -------------------------------------------------------------------------
#Copyright © 2021 Wei-Chih Lin , kjoelovelife@gmail.com 

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
# https://chtseng.wordpress.com/2019/05/01/nvida-jetson-nano-%E5%88%9D%E9%AB%94%E9%A9%97%EF%BC%9A%E5%AE%89%E8%A3%9D%E8%88%87%E6%B8%AC%E8%A9%A6/
#
# -------------------------------------------------------------------------

sudo apt-get update

#======= ste1. Enable i2c ===============
# Enable i2c permissions
sudo usermod -aG i2c $USER
#===========================================

#======== Step2. apt update and upgrade ==============================
sudo apt-get update
sudo apt install -y python3-pip python3-pil build-essential libssl-dev libffi-dev python3-dev
sudo pip3 install cython
#==============================================================

#======== Step3. install traitlets and jupyterlab  ============
sudo apt -y install curl dirmngr apt-transport-https lsb-release ca-certificates
curl -sL https://deb.nodesource.com/setup_12.x | sudo -E bash -
sudo apt -y install nodejs node-gyp 'gcc' g++ 'make'
sudo pip3 install jupyter jupyterlab
sudo python3 -m pip install git+https://github.com/ipython/traitlets@4.x
sudo jupyter labextension install @jupyter-widgets/jupyterlab-manager
sudo jupyter labextension install @jupyterlab/statusbar
#jupyter lab --generate-config
#echo $PASSWORD | jupyter notebook password
#echo $PASSWORD
# if jupyter notebook has the error : " bash: jupyter: command not found "
# can enter this command to solve : " pip3 install --upgrade --force-reinstall jupyter note  book "
# if you have problem with "get 403 ..." , can install ipykernel with this text : sudo python3 -m pip install ipykernel --user
#=============================================================

#========= step7. install repo of jetbot and configure jetbot service ==================	 
echo $PASSWORD | sudo apt install python3-smbus cmake
python3 create_jupyter_service.py
echo $PASSWORD | sudo mv jetbot_jupyter.service /etc/systemd/system/jetbot_jupyter.service
echo $PASSWORD | sudo systemctl enable jetbot_jupyter
echo $PASSWORD | sudo systemctl start jetbot_jupyter
#sudo chmod 777 /usr/local/lib/python3.6/dist-packages/jetbot-0.3.0-py3.6.egg/jetbot/* -R
#=======================================================================================

# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

#===== setup wifi SSID ===============
#sudo apt install network-manager
#sudo systemctl start NetworkManager.service
#sudo systemctl enable NetworkManager.service
## disconnect wifi
#sudo nmcli device disconnect wlan0

## rescan wifi 
#sudo nmcli device wifi rescan

## list wifi
#sudo nmcli device wifi list

## connect wifi
#sudo nmcli device wifi connect "SSID" password "SSID password"

#============================================

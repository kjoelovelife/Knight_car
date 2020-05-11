#!/bin/bash

###########
# reference : https://edimax.freshdesk.com/support/solutions/articles/14000065859-how-to-install-ew-7822ulc-ew-7822utc-in-linux-running-kernel-higher-than-v4-15
###########

set -e

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

set -x

sudo apt update
sudo apt install -y build-essential git
sudo apt install -y linux-headers-$(uname -r)

git clone https://github.com/EntropicEffect/rtl8822bu.git
cd rtl8822bu/
make
sudo make install

echo 'Please reboot to enable ew-7822ulc!'




# None of this should be needed. Next time you think you need it, let me know and we figure it out. -AC
# sudo pip install --upgrade pip setuptools wheel

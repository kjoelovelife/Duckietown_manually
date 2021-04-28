#!/usr/bin/env bash

# Shell script scripts to setup project "duckietown DB21M"
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
# Reference: https://docs.duckietown.org/daffy/
#            https://docs.duckietown.org/daffy/opmanual_duckiebot/opmanual_duckiebot-ready-tablet.pdf
# -------------------------------------------------------------------------

if [[ `id -u` -eq 0 ]] ; then
    echo "Do not run this with sudo (do not run random things with sudo!)." ;
    exit 1 ;
fi

sudo apt-get update 
sudo apt-get install bison autoconf flex
sudo apt-get install gcc-avr binutils-avr gdb-avr avr-libc avrdude
sudo apt-get install build-essential

cd ~/duckietown/DB21M/install_script/hardware/software/_avrdudeconfig
sudo cp avrdude.conf /etc/avrdude.conf

cd ~/duckietown/DB21M/install_script/hardware/software
make fuses
make clean

wget http://download.savannah.gnu.org/releases/avrdude/avrdude-6.2.tar.gz

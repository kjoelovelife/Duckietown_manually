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

#==== step 1. Install Ubuntu ====  
sudo apt update


#==== step 2. Basic dependencies ====
## install pip, git, git-lfs, curl, wget:
sudo apt install -y python3-pip git git-lfs curl wget

#echo -n "Please type your password: "
#read password

#==== setp3. Docker ====
## Install Docker
## reference: https://docs.docker.com/engine/install/ubuntu/
## step 3-1. Uninstall old versions
#sudo apt-get remove docker docker-engine docker.io containerd runc

## step 3-2. Supported storage drivers
## reference: https://docs.docker.com/storage/storagedriver/aufs-driver/

## step 3-3. Install using the repository
## Most users set up Docker’s repositories and install from them, for ease of installation and upgrade tasks. This is the recommended approach.
### step 3-3-1. Set up the repository
#### step 3-3-1-1. Update the apt package index and install packages to allow apt to use a repository over HTTPS:
sudo apt-get update
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common

#### step 3-3-1-2. Add Docker’s official GPG key:
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -

##### Verify that you now have the key with the fingerprint 9DC8 5822 9FC7 DD38 854A  E2D8 8D81 803C 0EBF CD88, by searching for the last 8 characters of the fingerprint.
sudo apt-key fingerprint 0EBFCD88

#### step 3-3-1-3. Use the following command to set up the stable repository. To add the nightly or test repository, add the word nightly or test (or both) after the word stable in the commands below.
#### Remember the struct(x86_64/amd64, armhf, arm64)
sudo add-apt-repository \
    "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) \
    stable"

### step 3-3-2. Install Docker Engine
#### step 3-3-2-1. Update the apt package index, and install the latest version of Docker Engine and containerd.
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io

#### step 3-3-2-2. Verify that Docker Engine is installed correctly by running the hello-world image.
sudo docker run hello-world

## step 3-4. Adds user to “docker” group:
sudo adduser $USERNAME docker

## step 3-5. Make sureyou have docker-compose installed:
sudo apt-get -y install docker-compose

#==== Step4. Duckietwon Shell ====
## reference: https://github.com/duckietown/duckietown-shell
## Note: Duckietown Shell required Python 3.6 or higher.
## Step 4-1. update pip
#pip3 install -U pip

## Step 4-2. Add yourself to the docker group. We have done it in step 3-4.
## Step 4-3. Install the duckietown-shell Python package:
#pip3 install --no-cache-dir --user -U duckietown-shell
pip3 install --user -U duckietown-shell
echo "export PATH=$PATH:~/.local/bin" >> ~/.bashrc
source ~/.bashrc
rm -rf ~/.dt-shell

## Then set the version, now we use the currently version "daffy" 
dts --set-version daffy

### step 4-3-1. Testing the Installation
which dts

#==== step 5. Other suggested configuration ====
## other userful packages
#sudo apt install -y vim byobu openssh-server nfs-common zsh
#echo "export EDITOR=emacs" >> ~/.profile

#==== step 6. Z shell ====
## step 6-1. setup zsh
#chsh -s /usr/bin/zsh

## step 6-2. Install oh-my-zsh
#sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-myzsh/master/tools/install.sh -O -)"

## step 6-3. set a different theme using
#echo "export ZSH_THEME"="bureau" >> ~/.profile

echo "====================================="
printf "\E[0;33;40m"
echo "Awesome! Now you can visit https://www.duckietown.org/site/register to register on the Duckietown website."
echo "In Taiwan, you can visit this webist to register: https://taiwan.duckietown.org/taiwan?lang=zh-hant"
echo "After register, you can find your Duckietown token, will allows to authenticate your devices to the Duckietown net-work."
echo "The token is a string of letters and numbers that looks something like this:"
echo "dt1-7vEuJsaxeXXXXX-43dzqWFnWd8KBa1yev1g3UKnzVxZkkTbfSJnxzuJjWaANeMf4y6XSXBWTpJ7"
echo "If you had a token, then you can type [\$dts tok set] and follow the prompt to get correctly identified as uid."
echo "Great! If you want to view your account for information of AIDO, you can type the code [\$ dts challenges info] to see"
echo "Finally if you don't have accounts for Github and DockerHub, please apply by :"
echo "Github: https://github.com/join"
echo "DockerHub: https://hub.docker.com/signup"
echo "Awesome! Please type \"dts\" to update duckietown-shell"
printf "\E[0m"
echo "====================================="

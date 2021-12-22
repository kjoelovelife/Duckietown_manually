#!/usr/bin/env bash

# Shell script scripts to burn the SD card for project duckietown
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

#==== step 1. Information need to know ====
printf "\E[0;33;40m"

echo "An SD card of size at least 32GB, and make sure your machine can read the SD card."
echo "A computer with a Ubuntu OS (for flashing the SD card), an internet connection, an SD card reader, and 16 GB of free space.
"
echo "Duckietown Shell, Docker, etc, has alerady configured by [laptop_ubuntu1804_setting.sh]."
echo "Duckietown Token set up will need. If not, please visit https://www.duckietown.org/site/register to register."
echo "And use [\$dts tok set] to set up the token."
echo "A correctly configured Duckiebot SD card in configuration DB18 (including Jetson Nano configurations)." 
echo" After assembling the Duckiebot, this will allow you to start it, connect to the internet,
and get going"
echo "Warning: this currently only works on Ubuntu. Mac is not supported."
printf "\E[0m"

echo ""
echo -n "Have you read infromation above? (y/N): "
read have_read_information

duckiebot_hostname=""
duckiebot_username="duckie"
duckiebot_password="quackquack"
duckiebot_SSID="duckietown"
duckiebot_SSID_password="quackquack"
duckiebot_country="US"
duckiebot_configuration=""
image_type="duckiebot"
burn_to_device=""

if [ "$have_read_information" '==' "y" ] || [ "$have_read_information" '==' "Y" ];
then
#####==== step 2. set the parameter for duckiebot ====
    #==== step 2-1. set hostname ====
    echo "Please set a hostname of the device to flash, this is required. "
    echo -n "Hostname for duckiebot: "
    read duckiebot_hostname

    #==== step 2-2. set linux username and password ====
    #echo -n "Do you want to set the linux user and password(default-> username:$duckiebot_username, password:$duckiebot_password)?(y/N): "
    #read set_user_password
    #if [ "$set_user_password" '==' "y" ] || [ "$set_user_passwd" '==' "Y" ];
    #then
    #    echo -n "Username: "
    #    read duckiebot_username
    #    echo -n "Password for username: "
    #    read duckiebot_password
    #else
    #    echo "Skip set linux user and password. Use default: username -> duckie, password -> quackquack"
    #fi

    #==== step 2-3. set SSID and password ====
    echo -n "Do you want to set the Wi-Fi SSID and password(default is [duckietown:quackquack])?(y/N): "
    read set_SSID
    if [ "$set_SSID" '==' "y" ] || [ "$set_SSID" '==' "Y" ];
    then
        echo -n "SSID: "
        read duckiebot_SSID
        echo -n "Password for SSID: "
        read duckiebot_SSID_password
    else
        echo "Skip set Wi-Fi SSID. Juse use the [SSID: duckietown, paswword: quackquack]."
    fi
    
    #==== step 2-4. set country ====
    echo -n "Do you want to set your country(default is [US])?(y/N): "
    read set_country
    if [ "$set_country" '==' "y" ] || [ "$set_country" '==' "Y" ];
    then
        echo -n "Your country: "
        read duckiebot_country
    else 
        echo "Skip set country. Use the default country [US]."
    fi

    #==== step 2-5. set type ====
    echo -n "Do you want to set the type(default is [duckiebot])?(y/N): "
    read set_type
    if [ "$set_type" '==' "y" ] || [ "$set_type" '==' "Y" ];
    then
        echo -n "Type(duckiebot, watchtower, traffic_light): "
        read image_type
    else
        image_type="duckiebot"
        echo "Skip set the type. Use the default [duckiebot]."
    fi

    #==== step 2-6. set configuration ====
    if [ "$image_type" '==' "duckiebot" ];
    then
        echo -n "Please select configutation of your duckiebot(DB21M, DB19, DB18): "
        read duckiebot_configuration
    else
        echo ""
    fi
    
    #==== step 2-7. select device to burn ====
    echo "Please select your device for burning the SD card:"
    lsblk
    echo -n "Burn the image to: /dev/"
    read burn_to_device
    burn_to_device=/dev/"$burn_to_device"

    #==== step 2-8. check out information ====
    echo "========== Information of image =========="
    echo "Username: $duckiebot_username"
    echo "Password: $duckiebot_password"
    echo "SSID: $duckiebot_SSID"
    echo "SSID Password: $duckiebot_SSID_password"
    echo "Country: $duckiebot_country"    
    echo "Image_type: $image_type"
    echo "Configuration: $duckiebot_configuration"
    echo "Device for burning SD card: $burn_to_device"
    echo "========== end of information =========="

    printf "\E[0;33;40m"
    echo -n "Are theese information right?(n/Y): "
    printf "\E[0m"

    read information
    if [ "$information" '==' "y" ] || [ "$information" '==' "Y" ];
    then
        echo "Please remember information of image. Start burning!"
        dts init_sd_card --hostname $duckiebot_hostname \
                         --wifi $duckiebot_SSID:$duckiebot_SSID_password \
                         --country $duckiebot_country \
                         --type $image_type \
                         --configuration $duckiebot_configuration \
                         --device $burn_to_device \
                         --no-cache
                         #--linux-username $duckiebot_username \
                         #--linux-password $duckiebot_password \
    else
        echo "Seems like some information is wrong. Please try again."
    fi
else
    echo "Skip Burning the SD card."
fi


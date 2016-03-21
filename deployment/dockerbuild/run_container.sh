#!/bin/bash

XAUTHORITY_FILE="/home/aurel/.Xauthority"

#xhost + #disable xhost auth

docker run --rm --name craftui --privileged \
    -v /dev/bus/usb/:/dev/bus/usb \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY_FILE:/root/.Xauthority \
    -v /home/aurel/craftui_volume:/mnt/ \
    -ti craftui 

#xhost - #enable xhost auth

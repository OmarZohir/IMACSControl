#!/bin/sh
#read -p "Please set correctly the WEBOTS_PATH, OPENCV_PATH, PKG_CONFIG_PATH and EIGEN_PATH in *.pro first. Press 'enter' after setting the paths."
echo "============================ STARTING COMPILATION ==============================="
qmake imacs_webots_framework.pro
make
read -p "============================= COMPILATION COMPLETED ================================ `echo $'\n '`If there is a compilation error, 'Ctrl+Z', rectify the error and then rerun this bash script.`echo $'\n '`If compilation completed without any errors, Close/Stop simulation of any open Webots instances. Press 'enter' after closing/stopping all Webots scenes."
#killall -9 webots-bin
#gnome-terminal -- sh -c 'cd webots_scenes; webots city.wbt; exec bash'
#gnome-terminal -- sh -c 'cd webots_scenes; webots city_straight.wbt; exec bash'
gnome-terminal -- sh -c 'cd webots_scenes; webots nightoriginal150.wbt; exec bash'
read -p "Press 'enter' after the webots scene has been opened."
./imacs_webots 1


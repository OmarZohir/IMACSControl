#!/bin/sh
#read -p "Please set correctly the WEBOTS_PATH, OPENCV_PATH, PKG_CONFIG_PATH and EIGEN_PATH in *.pro first. Press 'enter' after setting the paths."
clear
echo "============================ STARTING COMPILATION ==============================="
qmake imacs_vrep_framework.pro
make
read -p "============================= COMPILATION COMPLETED ================================ `echo $'\n '`If there is a compilation error, 'Ctrl+Z', rectify the error and then rerun this bash script.`echo $'\n '`If compilation completed without any errors, Close/Stop simulation of any VREP instances. Press 'enter' after closing/stopping all VREP scenes"
#killall -9 coppeliaSim
#gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/EnterCurveTest.ttt; exec bash'
gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/SmallBias_report.ttt; exec bash'
read -p "==================================================================================== `echo $'\n '`Press 'enter' after the vrep scene has been opened"
echo "============================ STARTING SIMULATION ==============================="
./imacs_vrep 1
echo "============================ COMPLETED SIMULATION ==============================="

#!/bin/sh
for (( ; ; ))
do
	clear
	echo "====================== Enter an option from the following: ======================"
	echo "	0: QUIT."
	echo "	1: IMACS with WEBOTS."
	echo "	2: IMACS with VREP."
	echo -n "Option: "
	read IMACS_OPTION
	if [[ $IMACS_OPTION -eq 0 ]]
	then
		exit 0
	elif [[ $IMACS_OPTION -eq 1 ]] || [[ $IMACS_OPTION -eq 2 ]]
	then	
		compiledFlag=0		
		for (( ; ; ))
		do
			if [ $compiledFlag -eq 0 ]
			then
				echo "============================ STARTING COMPILATION ==============================="
				if [ $IMACS_OPTION -eq 1 ]
				then	
					qmake imacs_webots_framework.pro
				else
					qmake imacs_vrep_framework.pro
				fi
				make
				echo ""
				echo "============================ COMPILATION COMPLETED =============================="
				echo ""
				echo "If there is a compilation error, quit this script, rectify the error and then rerun this bash script."
				echo ""
				compiledFlag=1
			fi
			echo "====================== Enter an option from the following: ======================"
			echo "	0: QUIT."
			if [ $IMACS_OPTION -eq 1 ]
			then	
				echo "	1: SIMULATE IMACS WEBOTS. Choose this option ONLY if a webots scene is already open."
				echo "	   If a simulation was running in WEBOTS, Reset Simulation (Ctrl+Shift+T) inside the open webots scene first."
				echo "	2: OPEN the webots-scene (CityStraight). Choose this option if no webots scene is open."
				echo "	3: OPEN the webots-scene (CityStraightNight). Choose this option if no webots scene is open."
				echo "	4: KILLALL webots scenes. The terminal needs to be manually closed though."
			else
				echo "	1: SIMULATE IMACS VREP. Choose this option ONLY if a vrep scene is already open."
				echo "	   If a simulation was running in VREP,stop simulation first."
				echo "	2: OPEN the vrep-scene (StraightRoad). Choose this option if no vrep scene is open."
				echo "	3: OPEN the vrep-scene (CurveRoad). Choose this option if no vrep scene is open."
				echo "	4: KILLALL vrep scenes. The terminal needs to be manually closed though."
			fi
			echo "	5: OPEN live_plot for visualisation."
			echo "	6: MAKE CLEAN and recompile."
			echo "	7: GENERATE doxygen documentation."
			echo "	8: CLEAN doxygen documentation."
			echo "	9: QUIT to IMACS WEBOTS/VREP option menu."
			echo -n "Option: "
			read OPTION
			if [ $OPTION -eq 0 ]
			then
				exit 0
			elif [ $OPTION -eq 1 ]
			then
				clear
				echo -n "Enter scenario: "
				read SCENARIO
				echo "============================ STARTING SIMULATION ==============================="
				if [ $IMACS_OPTION -eq 1 ]
				then
					./imacs_webots $SCENARIO
				else
					./imacs_vrep $SCENARIO
				fi
				echo ""
				echo "============================ COMPLETED SIMULATION =============================="
				read -p "Enter to continue."
			elif [ $OPTION -eq 2 ] 
			then
				if [ $IMACS_OPTION -eq 1 ]
				then
					gnome-terminal -- sh -c 'cd webots_scenes; webots city_straight.wbt; exec bash'
				else
					gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/StraightRoad.ttt; exec bash'
				fi
			elif [ $OPTION -eq 3 ]
			then
				if [ $IMACS_OPTION -eq 1 ]
				then
					gnome-terminal -- sh -c 'cd webots_scenes; webots nightoriginal150.wbt; exec bash'
				else
					gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/CurveRoad.ttt; exec bash'
				fi
			elif [ $OPTION -eq 4 ]
			then
				if [ $IMACS_OPTION -eq 1 ]
				then
					killall -9 webots-bin
				else
					killall -9 coppeliaSim
				fi
			elif [ $OPTION -eq 5 ]
			then				
				gnome-terminal -- sh -c 'python live_plot.py; exec bash'
			elif [ $OPTION -eq 6 ]
			then
				make clean
				compiledFlag=0		
			elif [ $OPTION -eq 7 ]
			then
				make doc		
			elif [ $OPTION -eq 8 ]
			then
				make clean_doc			
			elif [ $OPTION -eq 9 ]
			then
				break		
			else
				clear
			fi
			clear
		done
	else
		clear
	fi
done

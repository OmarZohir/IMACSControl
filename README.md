#Starting IMACS
To start IMACS VREP:
```
bash run_imacs_vrep.sh
```
Follow the instructions.

To start IMACS WEBOTS:
```
bash run_imacs_webots.sh
```

#IMACS Documentation
The main page of the doxygen documentation is `~/imacs/doc/mainpage.dox`.
The settings for the doxygen documentation is located at `~/imacs/doc/Doxyfile`.
You can generate doxygen documentation for IMACS code using the following command.
```
make doc
```
This will generate a `doxygen_output` folder inside doc. 
The entire doxygen documentation can be opened in a webpage by opening `~/imacs/doc/doxygen_output/html/index.html`.
To clean the documentation, 
```
make clean_doc
```

#Run IMACS with a different scene
If you want to change the VREP/WEBOTS scene to simulate, change it in `run_imacs_vrep.sh`/`run_imacs_webots.sh`.

##Change a VREP scene
1. Copy the scene (e.g. `*.ttt`) to `~/imacs/vrep_scenes` folder.
2. In `run_imacs_vrep.sh`, change the line `gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/SmallBias_report.ttt; exec bash'` to the following
```
gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/`*.ttt`; exec bash'

##Change a WEBOTS scene
1. Copy the scene (e.g. `*.wbt`) to `~/imacs/webots_scenes` folder.
2. In `run_imacs_webots.sh`, change the line `gnome-terminal -- sh -c 'cd webots_scenes; webots city_straight.wbt; exec bash'` to the following
```
gnome-terminal -- sh -c 'cd webots_scenes; webots `*.wbt`; exec bash'
```

#Run IMACS with different scenario
Note that 

# IMACS: A performance evaluation framework for IMAge in the Closed-loop Systems

# Starting IMACS
To start IMACS VREP:
```
bash run_imacs_vrep.sh
```
Follow the instructions.

To start IMACS WEBOTS:
```
bash run_imacs_webots.sh
```
# How to use IMACS
1. The controller needs to be designed separately from this toolchain.  
In the current framework, we use a lane keeping assist system (LKAS) example.  
The controller for this is implemented in `src/lateralController/`.  
The current `lateralControl*.*` files implement a lateral controller with sensor-to-actuator delay less than or equal to sampling period. This means that there is one augmented system state.  
If you want to implement a new controller, e.g. pipelined controller, LQG controller, etc., you need to write your own code for `lateralControl*.cpp` and `lateralControl*.hpp`. Further, if delay > period, the `actuate_steering_angle` in `main_IMACS_*.cpp` needs to be updated correctly considering the timing. 

2.
# IMACS Documentation
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
# To use the local Git
Before you use the git, please set your name and email.
```
git config --global user.email "you@example.com"
git config --global user.name "Your Name"
```
It is better to create your own branch and commit to master branch when you have a working improvement.

# Run IMACS with a different scene
If you want to change the VREP/WEBOTS scene to simulate, change it in `run_imacs_vrep.sh`/`run_imacs_webots.sh`.

## Change a VREP scene
1. Copy the scene (e.g. `*.ttt`) to `~/imacs/vrep_scenes` folder.
2. In `run_imacs_vrep.sh`, change the line `gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/SmallBias_report.ttt; exec bash'` to the following
```
gnome-terminal -- sh -c 'cd externalApps/vrep; ./coppeliaSim.sh ../../vrep_scenes/`*.ttt`; exec bash'

## Change a WEBOTS scene
1. Copy the scene (e.g. `*.wbt`) to `~/imacs/webots_scenes` folder.
2. In `run_imacs_webots.sh`, change the line `gnome-terminal -- sh -c 'cd webots_scenes; webots city_straight.wbt; exec bash'` to the following
```
gnome-terminal -- sh -c 'cd webots_scenes; webots `*.wbt`; exec bash'
```

# Run IMACS with different scenario
In `run_imacs_*.sh`, change the line `./imacs_* 1` to `./imacs_* <unsigned int scenario>`.
Note that the `period_ms`, `tau_ms` and the controller matrices `m_phi`, `m_Gamma`, `m_T` and `m_K2c` in `src/lateralController/lateralControl*.hpp` for the corresponding `<scenario>` needs to be updated. Else, it will throw and error.




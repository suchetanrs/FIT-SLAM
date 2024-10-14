This respository contains all the packages required to launch the Frontier based exploration in a 3D environment using traversability mapping.

# Setup instructions

1. Setup the simulation from https://github.com/suchetanrs/gz-sim-environment. The README contains the instructions for setup.
2. Clone this repository using ```git clone https://github.com/suchetanrs/FIT-SLAM && cd FIT-SLAM```.
3. Update the submodules using ```git submodule update --init --recursive```
4. We use a multi-stage docker build process to run both the SLAM and the exploration in the same container.
Setup the ORB-SLAM3 docker image and the exploration Docker image.
```sh
cd ORB-SLAM3-ROS2-Docker
sudo docker build -t orb-slam3-humble:22.04 .
cd ..
sudo docker build -t fit-slam:22.04 .
``` 
This can take quite a while! You might need a ☕. You may get a few warnings from rosdep. You can safely ignore them as long as it installs the packages.

5. Build the ORB-SLAM3 wrapper and the exploration packages.
5a. ```sudo docker compose run fit-slam```
5b. ```./build_packages.sh && exit # run once inside the container```

# To run exploration with Ground Truth localisation.

1. ```cd``` into the simulation git repository that you previously setup.
2. ```sudo docker compose run vehicle_simulator_gz_sim```
3. ```./launch_simulation.sh```
In a new terminal: <br>
```cd``` into the ```FIT-SLAM``` repository. <br>
```sudo docker compose run fit-slam``` <br>
Launch the exploration using: <br>
```./launch_exploration.sh```

# To run Active SLAM

1. ```cd``` into the simulation git repository that you previously setup.
2. ```sudo docker compose run vehicle_simulator_gz_sim```
3. ```./launch_simulation.sh```
In a new terminal: 
```cd``` into the ```FIT-SLAM``` repository. <br>
```sudo docker compose run fit-slam``` <br>
Launch the exploration using: <br>
```./launch_active_slam.sh```

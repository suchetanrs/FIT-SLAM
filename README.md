This respository contains all the packages required to launch the Frontier based exploration in a 3D environment using traversability mapping.

# Docker Setup

Clone this repository using: ```git clone -b ros2-humble https://gitlab.isae-supaero.fr/navires/pnx/students-codes/active-slam-suchetan```

```cd <cloned_repository>```

To launch the software, run the following commands.

```1) sudo chmod +x docker_install.sh```

```2) ./docker_install.sh```

This ensures the installation of docker on your machine.

```3) xhost +```

```4) sudo docker build -t ubuntu-ros2-humble-base:22.04 .```

This process might take sometime depending on your machine...Ensure you have an active internet connection...


# Launching the exploration

To launch the docker container containing the exploration software, run:

```4) sudo docker-compose run ros2-humble-base:22.04```

This will take you inside the docker container. Run the command ```login``` with username as ```suchetan``` and password as ```u``` once inside the container.

The container uses tmux. You can create new terminal panes by the following shortcuts.

```New pane to the right: 1. Ctrl+b 2. |```

```New pane to the bottom: 1. Ctrl+b 2. -```

The exploration requires 2 active terminals.

On one terminal run ```./all_nodes.sh```

Once all the nodes are up and running in terminal 1, on the second terminal run ```ros2 run frontier_exploration polygon_point_publisher```

In case the exploration does not start even after a couple of minutes, drive the robot using the teleop window forward into an explored area, this should start the exploration.
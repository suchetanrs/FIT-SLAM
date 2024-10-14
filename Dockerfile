FROM orb-slam3-humble:22.04

#setup
ENV DEBIAN_FRONTEND="noninteractive"

COPY ./root_dir/shell_scripts/basic_util.sh /root/
RUN cd /root/ && sudo chmod +x * && ./basic_util.sh && rm -rf basic_util.sh
COPY ./root_dir/shell_scripts/vscode_install.sh /root/
RUN cd /root/ && sudo chmod +x * && ./vscode_install.sh && rm -rf vscode_install.sh
COPY ./root_dir/shell_scripts/colcon.sh /root/
RUN cd /root/ && sudo chmod +x * && ./colcon.sh && rm -rf colcon.sh
COPY ./root_dir/shell_scripts/ros_package_install_essential.sh /root
RUN cd /root/ && sudo chmod +x * && ./ros_package_install_essential.sh && rm -rf ros_package_install_essential.sh
COPY ./root_dir/shell_scripts/exploration_dep_install.sh /root
RUN cd /root/ && sudo chmod +x * && ./exploration_dep_install.sh && rm -rf exploration_dep_install.sh
# COPY ./root_dir/shell_scripts/all-installed-packages.sh /root
# RUN cd /root/ && sudo chmod +x * && ./all-installed-packages.sh && rm -rf all-installed-packages.sh
# RUN . /opt/ros/humble/setup.sh && apt install ros-$ROS_DISTRO-foxglove-bridge && apt-get install -y ros-humble-rmw-cyclonedds-cpp
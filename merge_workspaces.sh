sudo chmod -R a+rw *
sudo chown -R $USER:$(id -gn) *

rm -rf ./dev_ws/src
cp -r ./root_dir/dev_ws/src ./dev_ws/src

rm -rf ./ros2_ws/src
cp -r ./root_dir/ros2_ws/src ./ros2_ws/src

rm -rf ./slam_ws/src
cp -r ./root_dir/slam_ws/src ./slam_ws/src

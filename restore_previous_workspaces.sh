sudo chmod -R a+rw *
sudo chown -R $USER:$(id -gn) *

rm -rf ./root_dir/dev_ws/src
cp -r ./dev_ws/src ./root_dir/dev_ws/src

rm -rf ./root_dir/ros2_ws/src
cp -r ./ros2_ws/src ./root_dir/ros2_ws/src

rm -rf ./root_dir/slam_ws/src
cp -r ./slam_ws/src ./root_dir/slam_ws/src

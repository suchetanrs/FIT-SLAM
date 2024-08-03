sudo chmod -R a+rw *
sudo chown -R $USER:$(id -gn) *

rm -rf ./dev_ws/src
cp -r ./root_dir/dev_ws/src ./dev_ws/src

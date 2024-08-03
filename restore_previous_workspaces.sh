sudo chmod -R a+rw *
sudo chown -R $USER:$(id -gn) *

rm -rf ./root_dir/dev_ws/src
mkdir -p ./root_dir/dev_ws
cp -r ./dev_ws/src ./root_dir/dev_ws/src

sudo docker buildx build --platform linux/amd64 --build-arg USE_CI=false --load -t orb-slam3-humble:22.04 .
sudo docker buildx build --platform linux/amd64 --build-arg USE_CI=false --load -t suchetanrs/fit-slam:amd-2.0 .

sudo docker buildx build --platform linux/arm64 --build-arg USE_CI=false --load -t orb-slam3-humble:22.04 .
sudo docker buildx build --platform linux/amd64 --build-arg USE_CI=false --load -t suchetanrs/fit-slam:arm-2.0 .
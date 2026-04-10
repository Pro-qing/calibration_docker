构建 image
docker build -t calibration_img .

运行doecker
./run_docker.sh

开启另外的docker环境终端
docker exec -it calibration_dev /bin/bash
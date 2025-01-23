# marta_bringup
clona recursivo


docker build -t marta_bringup:1.0 -f docker/Dockerfile .

docker run -it --net=host -e DISPLAY=$DISPLAY --gpus all marta_bringup:1.0
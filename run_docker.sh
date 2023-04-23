xhost +local:root 
docker container prune -f 
docker run --privileged --rm -it \
    --name="auto_jenga" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$XAUTH:$XAUTH" \
    --network host \
    -v "$(pwd)/src/jenga_packages:/home/ros_ws/src/jenga_packages" \
    -v "$(pwd)/bags:/home/ros_ws/bags" \
    auto_jenga bash

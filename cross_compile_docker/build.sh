cd /work && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release
scp -r install ubuntu@192.168.1.103:/home/ubuntu/ros_ws/
echo Done.
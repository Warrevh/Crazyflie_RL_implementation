xhost +
sudo docker run -it --env="DISPLAY" --net=host --rm --privileged\
	-v /dev/bus/usb:/dev/bus/usb\
	-v /dev/input:/dev/input\
	-v /home/$USER/crazyswarm2/crazyswarm2:/root/ros2_ws/src/crazyswarm2\
	-v /home/$USER/crazyswarm2/motion_capture_tracking:/root/ros2_ws/src/motion_capture_tracking\
	-v /home/$USER/crazyswarm2/config:/root/ros2_ws/src/crazyswarm2/crazyflie/config\
	-v /home/$USER/crazyswarm2/cf_nodes:/root/ros2_ws/src/cf_nodes\
	-v /etc/udev:/etc/udev\
	--name cs2_new cs2_new_img bash

FROM osrf/ros:humble-desktop
SHELL ["/bin/bash","-c"]

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y git\
                       vim\
                       python3-pip\
                       libxcb-xinerama0
RUN apt-get install libgl1-mesa-glx -y
RUN pip3 install --upgrade pip
RUN apt install libboost-program-options-dev libusb-1.0-0-dev -y
RUN pip3 install rowan
RUN pip3 install cflib transforms3d
RUN apt-get install ros-humble-tf-transformations -y
RUN cd ~/ &&\
	echo "source /opt/ros/humble/setup.bash &&\
	cd ~/ros2_ws &&\
	colcon build --symlink-install --packages-skip qualisys_driver &&\
	colcon build --symlink-install" >> ~/colcon_build.sh
RUN chmod +x ~/colcon_build.sh
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
#RUN     source /opt/ros/humble/setup.bash &&\
#	cd ~/ros2_ws &&\
#	colcon build --symlink-install --packages-skip qualisys_driver &&\
#	colcon build --symlink-install 

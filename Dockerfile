FROM osrf/ros:humble-desktop
SHELL ["/bin/bash","-c"]

RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install -y git\
                       vim\
                       python3-pip
RUN apt install -y libxcb-xinerama0
RUN pip3 install --upgrade pip
RUN apt install libboost-program-options-dev libusb-1.0-0-dev -y
RUN pip3 install cflib transforms3d
RUN apt-get install -y python3-tk
RUN pip3 install scipy
RUN pip3 install rowan
RUN apt-get install ros-humble-tf-transformations -y
RUN apt-get update -y && apt-get upgrade -y
RUN apt-get install ros-humble-rosbridge-server -y
#RUN cd ~ &&\
	#git clone -b krichardsson/qt6-v2 https://github.com/bitcraze/crazyflie-clients-python &&\
	#git clone -b jonasdn/flight_plan_tab https://github.com/bitcraze/crazyflie-clients-python &&\
 #       git clone https://github.com/bitcraze/crazyflie-clients-python &&\
#	cd crazyflie-clients-python &&\
#	pip3 install -e . 
	
RUN cd ~/ &&\
	echo "source /opt/ros/humble/setup.bash &&\
	cd ~/ros2_ws &&\
	colcon build --symlink-install --packages-skip qualisys_driver &&\
	colcon build --symlink-install" >> ~/colcon_build.sh
RUN chmod +x ~/colcon_build.sh
RUN echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
#RUN pip3 install cfclient

# CRAZYSWARM IN VISIONEN

This is a reporitory that mainly contains the submodules "crazyswarm2" and "motion_capture_tracking", but also instalation instrucktions and docker files, specifically to make the system run in Visionen.

## Docker and containers
This project is container based and uses Docker. Docker is a tool to create containers, which are isolated operating systems running on your host OS. You can think of it as a scaled down virtual machine but with same performace as your host OS. The reason why we use it in this case is to make installation process easier. Docker is available for both windows and linux. Windows will require you to do some aditional stuff to get graphics in containers to work. It is therefore recomended to use a computer with linux (any version will do).

Installing docker is simple:
```
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Using the contaier
Each time you start your container, simply run `./run.sh`. You exit the container by writing `exit`. If you wish to add another terminal window in the container, open a new terminal and then run `./terminai.sh`. The directories for the packages as well as the config directory are mapped to locations in the contaner. This means that changes you do in these directories in the container will also take place in that directory on the host, and vice versa.
NOTE: files created outside of these directories in the container will NOT be saved when you run `exit`.

## Installation
Clone the repo into your home directory. Use the recursive flag to include the submodules:
```
git clone https://gitlab.liu.se/visionen/ros2/crazyswarm2.git --recurse-submodules
```

To give permissions for the software to use the usb radio, we need to add udev rules. These can be added manually as dercribed here https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/
but there should also be a script in the crazyswarm directory that does this automatically.


```
cd ~/crazyswarm2/crazyswarm2
./pc_permissions.sh
```

Now it's time to build the container. Navigate back to the root of the repo.

```
cd ~/crazyswarm2
./build_docker_image.sh
```

To start the container, run:
```
./run.sh
```

When inside the container, run:

```
cd
./colcon_build.sh
source ros2_ws/install/setup.bash
```

## Usage

### Crazyflie
To do changes to the crazyflie, for example changing its radio address or flashing new firmware, you can use the crazyflie client. For some reason this client does not work in the container, so you will have to install it on your host. Installation instructions can be found here https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/installation/install/

You need to configure two things here: channel and addess. The reason is that you will have to define them later when running crazyswarm.

### Qualisys
Before you start flying, you need to define the crazyflie as a rigid body in Qualisys. How you do this is defined in the Qualisys guide. Remember to define the cf such that its x-axis is pointing forward. (the blue LEDs are in the back). A good convention is to name the crazyflie after its address. If the address is 0xE7E7E7E702 you should name it "cf2", if the address is 0xE7E7E7E701 you should name it "cf1" and so on.

### ROS
The last thing to do before launching crazyswarm is to connect to the visionen Wi-Fi. The name is "Visionen-5GHz". You need to be connected to this nework as this is where Qualisys streams the positioning data. Note that this network is not connected to the internet, but you will quickly realize this when you try to chatGPT your way out of your problems.

In the config folder you will find some files where you can set some parameters. The only one you need to bother with in the beginning is the _crazyflies.yaml_ file. Set the _name_ according to the name set in Qualisys, and the _uri_ to the correct channel and address and _type_ to "cf".

Now you should be good to go. Launch crazyswarm by typing
```
ros2 launch crazyflie launch.py
```

You should now see a Rviz window with the position of the crazyflie displayed. If the position shown in rviz does not correspond to the real position of the cf (or if it is non existent), do your crazyflie a favour and don't ask it to fly.

If everything looks good you can run the test script "hello_world.py" by writing `ros2 run crazyflie_examples hello_world` at which the crazyflie should take off, hover for a few seconds before landing again.

When writing your own scripts, they can preferably be put in the ros2_ws/src/cf_nodes directory. Take inspiration from what is written in the CMakeLists.txt file and you will figure out what to add to make an executable for your own node. Appart from the python scripts provided by bitcraze, there are also two nodes written in c++ in the cf_nodes package, to give you an idea of how these can look.

## Resources

If you are new to ROS, it is recomended to go through some of the tutorials here: http://docs.ros.org/en/humble/Tutorials.html

For more information about crazyswarm2 you can look at the documentation here: https://imrclab.github.io/crazyswarm2/

If you are interested to flash your own firmware (for example to add your own controller) instructions can be found here: https://www.bitcraze.io/documentation/repository/crazyflie-firmware/master/building-and-flashing/build/
and here: https://www.bitcraze.io/2023/02/adding-an-estimator-or-controller/

Intesesting reading: https://www.sciencedirect.com/science/article/pii/S2405896322010060

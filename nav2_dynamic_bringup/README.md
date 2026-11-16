# nav2_dynamic_bringup

The `nav2_dynamic_bringup` package is an example bringup system for launching CARLA to test navigation in a dynamic environment.

To setup a dynamic environment, you need to run the following scripts.

--------------

### Installing CARLA 0.9.13 on Ubuntu 22.04 

#### Installation Procedure (Docker)

Follow the link below and execute the provided script: https://carla.readthedocs.io/en/docs-preview/build_docker/
```shell
docker pull carlasim/carla:0.9.13
```

#### Installation Procedure (Native)
Alternatively, you can download CARLA manually from the following link:
https://github.com/carla-simulator/carla/releases/tag/0.9.13 and download `CARLA_0.9.13.tar.gz`.


-------------------


### Installing CARLA ROS Bridge (Docker)
```bash
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone --recurse-submodules https://github.com/chiragmakwana0296/ros-bridge.git -b fix/docker src/ros-bridge
cd src/ros-bridge/docker
docker compose build build_image
docker compose up colcon_build
``` 
#### Host setup
```bash
sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
# add below lines to ~/.bashrc
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=~/carla-ros-bridge/docker/content/dds.xml 
```


### Installing CARLA ROS Bridge (Native)

Clone the master branch of ```carla_ros_bridge``` and follow the official installation guide:
https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

```shell
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge

git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git src/ros-bridge

rosdep update
rosdep install --from-paths src --ignore-src -r

colcon build
```
#### Installing `.egg` and `.whl` Files
For running CARLA ROS Bridge on Ubuntu 22.04, you need compatible .egg and .whl files.
Download them from the following link: https://github.com/gezp/carla_ros/releases/tag/carla-0.9.13-ubuntu-22.04

Copy the downloaded files to the CARLA Python API directory:
* carla-0.9.13-cp310-cp310-linux_x86_64.whl 
* carla-0.9.13-py3.10-linux-x86_64.egg 

```shell
cp carla-0.9.13-cp310-cp310-linux_x86_64.whl /path_to_carla/CARLA_0.9.13/PythonAPI/carla/dist/
cp carla-0.9.13-py3.10-linux_x86_64.egg /path_to_carla/CARLA_0.9.13/PythonAPI/carla/dist/
```

#### Installing the `.whl` File
```shell
cd /path_to_carla/CARLA_0.9.13/PythonAPI/carla/dist/
pip3 install carla-0.9.13-cp310-cp310-linux_x86_64.whl 
```

#### Setting Up Environment Variables
Add the CARLA paths to your `.bashrc` file:
```shell
export CARLA_ROOT=/path-to-carla/CARLA_0.9.13/
export PYTHONPATH=$PYTHONPATH:$CARLA_ROOT/PythonAPI/carla/dist/carla-0.9.13-py3.10-linux-x86_64.egg:$CARLA_ROOT/PythonAPI/carla
```

Then, source the file:
```shell
source ~/.bashrc
```


### Running CARLA and CARLA-ROS Bridge

-------------------------------
#### Run CARLA Simulation (docker)
1. To run the CARLA simulation, execute the following command in Terminal 1:
```shell
sudo docker run --privileged --gpus all --net=host -e DISPLAY=:1 carlasim/carla:0.9.13 /bin/bash ./CarlaUE4.sh
```

#### CARLA ROS Bridge (Docker)
Run bridge 
```bash
docker compose up run_bridge
```
To generate thetraffic 
```bash
docker compose up generate_traffic
```

#### CARLA ROS Bridge (Native Ubuntu 22.04)
2. To run the CARLA ROS bridge, execute the following command in Terminal 2:
```shell
ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

3. To send command signals to CARLA, run the following command in Terminal 3:
```shell
ros2 run carla_twist_to_control carla_twist_to_control
```

4. To generate traffic, execute the following commands in Terminal 4:
```shell
cd /path_to_carla/CARLA_0.9.13/PythonAPI/examples/
python3 generate_traffic.py
```

#### Nav2 Stack for Navigation
5. To run the navigation stack, execute the following command in Terminal 5:
```shell
ros2 launch nav2_dynamic_bringup navigation_launch
```
## ROS-2 Programming Assignment_1 - Publisher/Subscriber

Hello! this is a ROS2 package 'beginner_tutorials' made for the ROS 2 Programming Assignment_1. 

### Dependencies and Requirements

- Ubuntu 22.04 (If running locally)
- Git
- C++17
- ROS2 Humble


### Build instructions

```bash
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/kashifansaricodes/my_beginner_tutorials.git
cd ..
colcon_build
source . install/setup.bash
```

### Running the publisher and subscriber node

```bash
cd ros2_ws/
source . install/setup.bash
ros2 run beginner_tutorials talker
```
In another terminal

```bash
cd ros2_ws/
source . install/setup.bash
ros2 run beginner_tutorials listener
```





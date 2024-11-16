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

### Running the launch file

```bash
cd ros2_ws/
source . install/setup.bash
ros2 launch beginner_tutorials tutorial_launch.py
```
### To change the string use

```bash
ros2 service call /change_string example_interfaces/srv/SetBool "data: true"
```

### To change the frequency use

```bash
ros2 param set /talker publish_frequency 5.0"
```
### To run the catch2 test (with additional log details)

```bash
colcon test --packages-select beginner_tutorials --event-handlers console_direct+
```




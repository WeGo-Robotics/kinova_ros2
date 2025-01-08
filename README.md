# kinova_ros2
kinova ros2 (python, cpp, moveit)

1. Download Kortex ros2

Kortex_ros2 : https://github.com/Kinovarobotics/ros2_kortex


2. git clone kinova_ros2
```
git clone https://github.com/WeGo-Robotics/kinova_ros2.git
```


3. if you use kinova gen3 lite
```
ros2 launch kortex_bringup gen3_lite.launch.py robot_ip:=192.168.1.10 #ethernet 192.168.2.10
```


4. play example (Execute the example in a non-packaging position)
```
ros2 run kinova_python zero_position
```
   

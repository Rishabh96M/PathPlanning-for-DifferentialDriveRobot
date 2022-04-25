Implemented on Python 3.8.10, Packages used are ROS Neotic, Math, OpenCV and NumPy

# PathPlanning-for-DifferentialDriveRobot
Navigate  a  differential  drive  robot  (TurtleBot  3)  in  a  given  map  environment from a given start point to a given goal point using A* algorithm.


## To clone the project
```
cd <catkin_ws>/src
git clone https://github.com/Rishabh96M/PathPlanning-for-DifferentialDriveRobot.git
cd ../
cd catkin_make
```

## To run project
Open a terminal,
```
roslaunch PathPlanning-for-DifferentialDriveRobot assignment.roslaunch
```
This will launch the turtlebot3 in a gazebo environment. <br>
<br>

In a new terminal,
```
cd <catkin_ws>/src/PathPlanning-for-DifferentialDriveRobot
python3 main.py
```
In put the values accordingly, All the units are in cm and degrees respectively.<br>
Map size is 1000cm * 1000cm<br>

Ideal Values:<br>
clearance : 0 - 5cm<br>
rpm1 : 10<br>
rpm2 : 20<br>

**Videos** folder contains the output for two test cases<br>
test1 inputs: clearance=0; start=100,100,0; goal=800,800; rpm=10,20<br>
test2 inputs: clearance=0; start=100,100,0; goal=300,500; rpm=10,20<br>

P.S: Try to run the videos on VLC Media player, and make sure it is completely zoomed out (The video contains two screens)

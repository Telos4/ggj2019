
# BRTA – Come Home
This is our game for **Global Game Jam 2019**.

![BRTA - Come Home](https://github.com/Telos4/ggj2019/blob/master/Art/Titelbild.png?raw=true)

**“BRTA – Come Home”** is a game about a rover named BRTA (Brave Rover inTelligence Advanced) built by IARA (International Aerospace Research Agency). On its way to a mission to the Galilean moons it crashes on planet Mars, losing all its memories in the process. On Mars BRTA needs to find supplies to get back to earth, while finding story-fragments that show why earth is very special to BRTA.  
The game is designed for two players, one that controls BRTA and another one who puts story-fragments, supplies and dangerous events in BRTA’s way.

The game is an **AR (Augmented Reality)** game in which you control a **rover** that is equipped with a **camera**. In addition, the rover has sensors for detecting **RFID cards**, as well as measuring **light** and **distance** to obstacles.  
ArUco markers are placed in the environment and are detected by OpenCV. You have to collect all markers in order to complete the game. This is made more difficult by dangers on RFID tags that are hidden on the floor and that can damage (= distort) the camera image.  
Driving around drains the battery of the rover and you have to complete all tasks before the battery is empty in order to win the game. Once you collect a solar panel, you can charge the battery again by driving the car under a light source.
![Rover](https://github.com/Telos4/ggj2019/blob/master/screenshots/IMG_2595.JPG?raw=true)
![Gameplay](https://github.com/Telos4/ggj2019/blob/master/screenshots/IMG_9420.jpg?raw=true)
![Item](https://github.com/Telos4/ggj2019/blob/master/screenshots/items.png?raw=true)

## Hardware and Software:
The rover is based on the [Berkely Autonomous Race Car](http://www.barc-project.com/) (BARC) project with some additional modifications.
It consist of an off-the-shelf RC car that has been equipped with an odroid, a camera and several other sensors. The communication with sensors and actuators of the car is handled by two Arduinos.  
On the software side of things we use [Robot Operating System](http://www.ros.org/) (ROS) to enable the control of the car and to provide access to sensors via network.
The game itself has been implemented in Python with the pygame library.

You play the game on a laptop that receives the camera and sensor streams from the car, runs the image detection and sends control commands to the car. You can either use an Xbox gamepad controller or a USB steering wheel and pedals (for more car feeling) to control the car.

## Installation instructions:
Hard- and Software are quite involved so installation is a bit tricky. Here we summarize the main steps.

We assume that the BARC project has been set up on the odroid according to the instructions on the [setup website](http://www.barc-project.com/setup/). In addition, ROS (version: Lunar) needs to be installed on the laptop and then the barc package needs to be build and installed as well (refer to the [ROS documentation](http://wiki.ros.org/Documentation) on how to install packages).

### **Copy source files to odroid:**  
Let $BARC denote the locations of the barc package (on the laptop and the odroid).  
Copy the files from the _odroid/_ subfolder to the home directory of the odroid (i.e. /home/odroid/).  
In addition, also copy the only the messages from the subfolder _odroid/barc/workspace/src/barc/msg_ to the message folder of the barc package on the laptop, i.e. _$BARC/workspace/src/barc/msg_.

### **Build ROS messages:**  
You now have to include the new messages in the file _$BARC/workspace/src/barc/CMakeLists.txt_ by adding 
```
    Echo.msg  
    Light.msg
    RFID.msg
```
    
to the command _add\_message\_files( ... )_ (both on the odroid and the laptop).  
Now build the package again by running the following commands in the _$BARC/workspace_ folder (on the odroid and on the laptop)
```
    $ catkin_make . 
    $ catkin_make install
```
Now the messages should be build and if you run
```
    $ rosmsg package barc
```
they should appear in the output. 

### **Arduino setup on the Odroid:**  
The odroid needs to be connected to two arduinos. The first Arduino handles communication with car actuators and encoder wheel sensors as described on the [BARC project page](http://www.barc-project.com/setup/).  
The second arduino is for reading the light, distance and RFID sensors. You have to connect the sensors and then flash the arduino sketch.  
The sketch requires headerfiles for the messages you just built. In order to create these, run the following commands on the odroid:
```
    $ cd ~/sketchbook/libraries/
    $ rm -rf ros_lib  # removes old header files
    $ rosrun rosserial_arduino make_libraries.py .
```
For the MFRC522 RFID reader we use a library from [this](https://github.com/miguelbalboa/rfid) Github repository.
You need to install it by placing the files _MFRC522.h_ and _MFRC522.cpp_ in _~/sketchbook/libraries/MFRC522_.

Now you should be able to flash the second arduino by using the script provided in the scripts/ subfolder. Just copy the script and run the script
```
    $ ~/barc/flash_nano_sensors
```
on the odroid.

**Note**: make sure that the serial ports for the two arduinos are correct (check the script!). We used _/dev/ttyUSB0_ for Arduino #1, and _/dev/ttyUSB1_ for Arduino #2.
Also make sure that the correct ports are configures in the launch file _$BARC/workspace/src/barc/launch/game.launch_ on the odroid.

You can now start the ROS programs on the odroid by running
```
    $ roslaunch barc game.launch
```
If everything went well, you should be able to see the sensor topics by running:
```
    $ rostopic list
```
and you can display for example the output from the light sensor by:
    $ rostopic echo /light 
The camera stream can be accessed by:
```
    $ rosrun image_view image_view image:=/image_raw _image_transport:= compressed
```
### **Laptop**  
On the laptop you need to install _Python 2.7_, _pygame_ (at last 1.9.4) and _numpy_. The program uses the libraries _rospy_, _cv2_ and _cv2.aruco_. On my system (Ubuntu 16.04) those were installed automatically when I installed ROS.

##How to run the game:
Launch game node on odroid: 
```
    $ ssh odroid  # log into the odroid
    $ roslaunch barc game.launch 
```
Launch game on laptop:
```
    $ python game_main.py
```

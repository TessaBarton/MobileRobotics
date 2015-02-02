# MobileRobotics
##Code for controlling an autonomous robot, on the Jaguar lite platform 
Jaguar is a 2 wheel differential drive robot, this program works on several aspects of robot localization, processing, and navigation
###Localization: 
Look at the functions in Navigation.cs:
-WallPositioning uses the onboard Laser Scanner to determine position, then uses proportional 
control to spin the left and right motors
-Odometry uses information from the wheel encoders to determine the speed and heading of the robot, this is how the robot is rendered on the GUI
###Processing:
The robot uses a map along with odometry, laser scan, and gps to statistically determine where it is most likely located
###Navigation:
All motor control is set by driving the left and right motors, slippage and terrain inconsistancies will be factored into the probabalistic location model
*******
##To run:
You can see how this program works even without the robot, just don't push the hardware mode button on the GUI, if you want to run the 
code on your own Jaguar, you need to set up the wireless connection (email me). 
Compile in Visual Studio and then run, you can move the imaginary robot (if not in hardware mode) around the GUI pane using the left and right 
trackpads, the robot doesn't pick up on speeds less than appx 20. 
Control loop in JaguarControl is called pretty much continuously so any autonomization functions 

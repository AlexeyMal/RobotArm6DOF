# RobotArm6DOF

 ![](https://github.com/AlexeyMal/RobotArm6DOF/blob/main/6DOF.jpg)
 
Robot Arm 6 DOF 
Arduino UNO 
HU-M16 joystick shield 
IR receiver (optional)
record and playback

Original source is here: 
https://projecthub.arduino.cc/utilstudio/mearm-161-robot-joystick-board-recording-movements-ir-235439
  
Extended by me to support 6 servos (added WristRoll and WristPitch) 
Uses two analogue joysticks and six servos.
  
Allows 4 modes: 
Manual mode 
Recording of coordinates - indicated by permanent light of LED diode. 
Play mode - play once recorded movements 
Play repeatedly - plays recorded movements repeatedly - indicated by flashing of LED diode. 

Hold the left joystick button to control WristRoll and WristPitch with the right joystick.
Press the button on joystick 1 to Start/Stop the recording. 
Press the button on joystick 2 to Start/Stop the playback. 
Press and hold button for 2 seconds on joistick 2 to Start/Stop the playback in a loop.


# fyp-Autonomous-model-car-based-on-arduino-and-Raspberry-Pi

This project aims to convert the autopilot technology from a real car into a model car which the model car should be built to achieve some autopilot functions. Autopilot technology was a brand-new technology springs up in recent years, and many barriers still exist to stop the development of this technology. The severest problem is a security problem. In order to test the security of the autonomous driving car, lot of fonds were put into the road test. In this report, a brand-new test method was proposed. An autonomous model car could be used to test the completeness of autopilot technology in a designed testing place. In this project, the model car contained four autonomous driving functions which are Cruise Control, Collision Prevention, Lane Keeping and Traffic Light Recognition. The front two functions were achieved by Arduino Mega 2560 and the rear two functions were completed in the computer. Python and OpenCV were involved in the image processing part of the project. In the end, the autonomous model car was tested in a geometric narrowed route and all functions run well.

![img](https://github.com/crazyggboom/fyp-Autonomous-model-car-based-on-arduino-and-Raspberry-Pi/blob/master/image_folder/model-car.jpg)

Test Vedio

![img](https://github.com/crazyggboom/fyp-Autonomous-model-car-based-on-arduino-and-Raspberry-Pi/blob/master/image_folder/test.gif)


## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites
HardWare

```
1.	Arduino Mega 2560 ReV3
2.	Raspberry Pi 3B+
3.	Raspberry Pi v2.1 8 MP 1080p Camera Module
4.	LM2596 DC to DC Voltage Regulator
5.	A4950 Full-Bridge DMOS PWM Motor Driver
6.	voltage Measurement Module
7.	SSD1306 0.92 inch 128*64 Pixel OLED Display Shield Board Module
8.	DC Geared Motor with Hall Current * 2
9.	2s 1900mah Lipo voltage
10.	2.4GHz Wireless PS2 Gamepad
11.	HC - 05 Bluetooth Module
12.	Anker Powercore 5000 Portable Charger
13.	HC – SR04 Ultrasonic Distance Sensor
14.	Printed Circuit Board
15.	RC Car Kit （https://item.taobao.com/item.htm?spm=a1z09.2.0.0.5c2c2e8dPeEeKL&id=556110269103&_u=8cvo7l2e174）

```
SoftWare

```
1.	Arduino IDE 1.8.7
2.	SSD1306.h – Library for OLED
3.	Servo.h – Library for Servo
4.	PinChangeInt.h – Library for external break
5.	MsTimer2.h – Library for timing break
6.	PS2X_lib.h – Library for PS2
7.	Raspbian – Raspberry Pi Operating System based on Debian (Kernel Version 4.14)
8.	JetBrains PyCharm Community Edition 2018.2 x64
9.	Qt Designer
10.	Python – 3.6
11.	PyQt5 – 5.11.3
12.	OpenCV-python – 3.4.3.18
13.	PySerial – 3.4
14.	Easygui – 0.98.1
15.	Numpy – 1.15.0
16.	Mjpg – Streamer (https://github.com/jacksonliam/mjpg-streamer.git)

```

### Installing

1. The Mjpg-Streamer should be added into the Raspberry
(https://github.com/jacksonliam/mjpg-streamer)
```
cap = cv2.VideoCapture('http://192.168.137.92:8080/?action=stream')
```
change the IP address due to your setting in Raspberry Pi

2. The lib in Arduino floder should be copied to the local library floder of Arduino IDE.

## Running the tests




### Controller Software

![img](https://github.com/crazyggboom/fyp-Autonomous-model-car-based-on-arduino-and-Raspberry-Pi/blob/master/image_folder/GUI.png)

### Lane Keeping

(1)	The image is resized into 600*600 pixel by function cv2.resize().

(2)	In order to distinguish the lane more obviously, the colorful image is converted into a grey one by function cv2.cvtColor.

(3)	A method named Histogram Equalization mentioned before is used to increase the contrast of the image. This method can highlight high-contrast part which is the difference between the light-color lane and dark-color road. The function used for Histogram Equalization is named cv2.qualizeHist in OpenCV.

(4)	The Blur pre-processing method is used to reduce the noise of the image. In this project, Gaussian Blur is used because Gaussian function can reduce more low-frequency points. 

(5)	Before using function cv2.findContours() to find the edge of the lane, cv2.threshold() is used to separate the interest region of the original image. The mode of counter retrieval algorithm is chosen as the RETR_TREE. Also, the method of contour approximation algorithm is chosen as the CHAIN_APPROX_SIMLPE. This method is to preserve the endpoints and compress the elements of horizontal, vertical and diagonal direction. 

(6)	Then, cv2.drawContours() is used to draw the contours found above and marked with green colors.

(7)	Also, in order to find the line of the lane, the Hough line transform is used to detect the line. The working principle was introduced before. 

(8)	The scope of the line determines the position of the car. If the angle of the lane is from -70 to -10, it means the car is too close to the left side. So, the car should turn left. On the contrary, if the angle is between 10 to 70, the car will turn left. In the other situation, it will keep straight. 

![img](https://github.com/crazyggboom/fyp-Autonomous-model-car-based-on-arduino-and-Raspberry-Pi/blob/master/image_folder/lane_keeping.png)

### Traffic Light Recognition

(1)	Firstly, the image is also resized into 600*600 pixel as previous.

(2)	The RGB color model is converted into HSV color model to detect the color.

(3)	Due to the table below, the range of different color is determined.

(4)	The upper matrix of red is set as [180, 255, 255] and the lower matrix is set as [156,43,46].

(5)	The upper matrix of red is set as [77, 255, 255] and the lower matrix is set as [35,43,46].

(6)	The function cv2.inRange() is used to separate the specified color region by above color matrix from the whole image.

(7)	In order to measure the distance between the camera and traffic light, the area of the traffic light is used to determine the distance. The colour region is firstly dilated using function cv2.dilate()

(8)	Then the contour area is calculated by the combination of function cv2.findContours() and function cv2.contourArea().

(9)	If the area of the color region is larger than a specific value, it means the model car has reached a specific position before the traffic light.

(10)	Therefore, the model car will decide the movement due to the color of the traffic light. Green represents move forward and Red represents stop.

![img](https://github.com/crazyggboom/fyp-Autonomous-model-car-based-on-arduino-and-Raspberry-Pi/blob/master/image_folder/Traffic_light_detection.JPG)


## Built With

* [ishanambike]https://github.com/ishanambike/Self-driving-Car-Using-Raspberry-Pi)

## Authors

* **Zining Jiang** - *Initial work* - [crazyggboom](https://github.com/crazyggboom)


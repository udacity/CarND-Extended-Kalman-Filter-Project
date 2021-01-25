Project: Extended Kalman Filter
---

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)

### Introduction
The major objective of this project is to implement a Kalman filter using *C++* as primay tool and tests with the [`Udacity simulator`](https://github.com/udacity/self-driving-car-sim). I'll be discussing following details to undestand few theoritical concepts that are used to implement this project.

* Sensors
* Kalman filter
* Extended Kalman filter
* Sensor fusion

| ![](data/images/dataset1.gif) | ![](data/images/dataset2.gif) |
|-------------------------------|-------------------------------|
|[*Dataset 01*](https://youtu.be/me-b7wNDdX4)| [*Dataset 02*](https://youtu.be/UQJ7q2JMteM)|

#### Sensors

*The three primary autonomous vehicle sensors are camera, radar and lidar. Working together, they provide the car visuals of its surroundings and help it detect the speed and distance of nearby objects, as well as their three-dimensional shape.  
In addition, sensors known as inertial measurement units help track a vehicle’s acceleration and location.* [Ref](https://blogs.nvidia.com/blog/2019/04/15/how-does-a-self-driving-car-see/)

##### Camera

*Autonomous vehicles rely on cameras placed on every side — front, rear, left and right — to stitch together a 360-degree view of their environment. Some have a wide field of view — as much as 120 degrees — and a shorter range. Others focus on a more narrow view to provide long-range visuals.* [Ref](https://blogs.nvidia.com/blog/2019/04/15/how-does-a-self-driving-car-see/)

|<img src="data/images/camera.png.webp" height="250" />|<img src="data/images/human-cross-road.jpg.webp" height="250" />|
|---------------------------------|---------------------------------------------------------------------------|
|Source: [How Does a Self-Driving Car See?](https://blogs.nvidia.com/blog/2019/04/15/how-does-a-self-driving-car-see/)|A camera sensor usecase|

##### Radar
*Radar sensors can supplement camera vision in times of low visibility, like night driving, and improve detection for self-driving cars.
Traditionally used to detect ships, aircraft and weather formations, radar works by transmitting radio waves in pulses. Once those waves hit an object, they return to the sensor, providing data on the speed and location of the object.* [Ref](https://blogs.nvidia.com/blog/2019/04/15/how-does-a-self-driving-car-see/)
|<img src="data/images/radar.jpeg" height="250" />|<img src="data/images/metawave.png.webp" height="250" />|
|---------------------------------|-------------------------------------------------------------------------------|
|Source: [Automotive Radar Simulation Software](https://www.remcom.com/automotive-radar)|A radar sensor usecase|


##### Lidar

*Lidar makes it possible for self-driving cars to have a 3D view of their environment. It provides shape and depth to surrounding cars and pedestrians as well as the road geography. And, like radar, it works just as well in low-light conditions.
By emitting invisible lasers at incredibly fast speeds, lidar sensors are able to paint a detailed 3D picture from the signals that bounce back instantaneously. These signals create “point clouds” that represent the vehicle’s surrounding environment to enhance safety and diversity of sensor data.* [Ref](https://blogs.nvidia.com/blog/2019/04/15/how-does-a-self-driving-car-see/)


|<img src="data/images/lidar-sensor.png.webp" height="250" />|<img src="data/images/laser.png.webp" height="250" />|
|---------------------------------|-------------------------------------------------------------------------------|
|Source: [How Does a Self-Driving Car See?](https://blogs.nvidia.com/blog/2019/04/15/how-does-a-self-driving-car-see/)|A lidar sensor usecase|

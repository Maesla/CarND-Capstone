
### Final Capstone Project - Term 3 - Self Driving Car Engineer Nanodegree

This is the project repo for the System Integration project of Term-3 Udacity Self-Driving Car Nanodegree.

The Team:

Manuel Espino Lara - espinolaramanuel@gmail.com - Team Lead <br />
Veeken Chaglassian - veeken77@gmail.com  <br />
Kyle Rector - kylearector@outlook.com  <br />
Nathaniel Owen - chris@chrisowen.org  <br />
Gary Lai - garylai2203@gmail.com  <br />


Here we also give a brief explanation for nodes and code for waypoint_updater and tl_detector.




### Objective

The project involves writing ROS nodes to implement the core functionality of the autonmous vehicle system such as following waypoints in a set path, using traffic light detection and control. The code will run in the simulator and be submitted to run on Carla, which is Udacity's autonmous test mule.


### Classification Model 

We experimented with several classification models to train the traffic light detector. Transfer learning with the coco dataset on a SSD-mobilenet graph was our final choice, in part because it is relatvely small as deep learning models go, a valid consideration when working with the hardware mounted in an car, in part because we had good annotated data to work for such a model (please see comments in code for references), and in part because we could hook it up to the Tensorflow object detection API with ease (not strictly necessary to do so, but still convenient).
We also created new datasets of our own (for the Tensorflow API expected the data to be annotated) and experimented with perhaps the most parsimonious of all conv net architectures, SqueezeNet (AlexNet-level accuracy with 50x fewer parameters and <0.5MB model size). While we achieved high levels of accuracy >97% on the validation set, testing on sim data was incomplete (for we'd have to generate new data sets here as well) and we didnt want to use the model without further testing. 



ssd-mobilenet:
https://research.googleblog.com/2017/06/mobilenets-open-source-models-for.html

ssd (Single-Shot-Multibox Detector) original paper:
https://arxiv.org/abs/1512.02325

MobileNet:
https://github.com/tensorflow/models/blob/master/research/slim/nets/mobilenet_v1.md

SqueezeNet orginal paper:
https://arxiv.org/abs/1602.07360

coco dataset:
http://cocodataset.org/#home


## Trafficlight detection - Single-Shot-Multibox Detector

The SSD is easy to train and strightforward to integrate into systems that have a detection component. This model can handle images with different resolutions for our traffic lights, as we start looking at the traffic lights from 100m. 

The traffic light detection node subsrcibes from 3 topics:
1. /base_waypoints
2. /current_pose
3. /image_colour

We use the vehicle's location and coordinates for traffic lights to find nearest visible traffic light ahead. The get_closest_waypoint does this by finding the closest waypoints to the vehicle and lights. This will determine which light is ahead of the vehicle.

Next we use the camera and our SSD network to classify the colour of the traffic light. Once we have classified our traffic light, in case of a red light detection, the event along with the location of the nearest traffic light stopping zone ahead of the vehicle, is published to the /traffic_waypoint.

Further details can be found in the code tl_classifier.py and tl_detector.py.

### Waypoint Updater


This waypoint updater is used for processing the many base points or nodes, that are static and retreiving a specific set of waypoints that represent the future set of points or nodes. The nodes may carry specific velocity values that account for any red lights that may have been detected by the car.

To start we need to add a subscriber for /traffic_waypoint and /obstacle_waypoint. We start by having a ROS parameter called /lookforward_ to see how many waypoints are needed. The ROS parameter LOOKAHEAD_WPS controls how many waypoints are included in the horizon set. If the red traffic light is published to the /traffic_waypoint, a velocity is generated in order to slow down at a decelaration rate and stops at the waypoint. A parameter called MAX_DECEL controls and adjusts the deceleration rate, so if we would like the vehicle to stop braking sooner, we make the value lower, wich will caculate the velocity profile with a lower decceleration rate which in turn and we also have a set BRAKE_DISTANCE which can also be adjusted if we require a longer braking distance. 

As in the Path Planning project, acceleration should not exceed 10 m/s^2 and jerk should not exceed 10 m/s^3.


Further details can be found in waypoint_updater.py.

## Braking

For the slow gradual braking we looked at several mathematical functions to implement, that allows a slow gradual stop. In reality, we would not want an abrupt stop as that may cause discomfort or even injure the passengers inside. In particular we looked at the easeOutCubic function.

Easing functions: (in Spanish)
http://easings.net/es


### Drive-By-Wire Node

The DBW node is responsible for publishing throttle, brake and steering set-point values for the car's lower level control system to execute. 
The dbw_node.py publishes steering, throttle, and brake commands at 50hz. 
Most of the node has been kept as simple as possible in order to help with the debugging and feature development that is likely to occur as real-world testing commences. 


### Known Issues 

Around two thirds of the track, around waypoint 6000 the car starts to behave more erratically. As the car's throttle suddenly goes far too high when accelerating from the traffic lights. This may be due to system resource allocation, reading on forums and on slack, we are not the only ones with this issue. 

There was an issue with the traffic light detection as the lights images taken from the camera may be too small (literally a few pixels wide).
Recent updates to TensorFlow last month may have also introduced some bugs, but using the older version of would barely classify the traffic light at 0.57 (threshold is 0.5)


For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images

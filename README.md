# About
![](https://raw2.github.com/arpesenti/peopleTracker/master/people.png)

Small-footprint mobile ground robots, such as the popular [Turtlebot](http://www.turtlebot.com) and [Kobuki](http://kobuki.yujinrobot.com) platforms, are by necessity equipped with sensors which lie close to the ground. Reliably detecting and tracking people from this viewpoint is a challenging problem, whose solution is a key requirement for many applications involving sharing of common spaces and close human-robot interaction. Here you can find a robust solution for cluttered indoor environments, using an inexpensive RGB-D sensor such as the Microsoft Kinect or Asus Xtion. A MATLAB real-time ROS-enabled implementation is available on this git repository.

# Publications
Please refer to the following publications describing our system.

**Kinect-based People Detection and Tracking from Small-Footprint Ground Robots**  
A. Pesenti Gritti, O. Tarabini, J. Guzzi, G. A. Di Caro, V. Caglioti, L. M. Gambardella, A. Giusti  
In Proc. International Conference on Intelligent Robots and Systems (IROS) 2014.  
Bibtex:
>@incollection{pesentigritti2014a,
>  booktitle={Proc. International Conference on Intelligent Robots and Systems (IROS) 2014},
>  title={Kinect-based People Detection and Tracking from Small-Footprint Ground Robots},
>  author={Armando Pesenti Gritti and Oscar Tarabini and Jerome Guzzi and Gianni A. Di Caro and Vincenzo Caglioti and Luca M. Gambardella and Alessandro Giusti}
>}

**Video: Perceiving People from a Low-Lying Viewpoint**
A. Pesenti Gritti, O. Tarabini, A. Giusti, J. Guzzi, G. A. Di Caro, V. Caglioti, L. M. Gambardella
In Proc. Human Robot Interaction (HRI) 2014.
Bibtex:
>@incollection{pesentigritti2014b,
>  booktitle={Proc. Human Robot Interaction (HRI) 2014},
>  title={Video: Perceiving People from a Low-lying Viewpoint},
>  author={Armando Pesenti Gritti and Oscar Tarabini and Alessandro Giusti and Jerome Guzzi and Gianni A. Di Caro and Vincenzo Caglioti and Luca M. Gambardella}
>}

# Getting Started
The system is implemented in MATLAB, with the most computationally expensive tasks written as mex functions able to exploit multi-core CPUs thanks to OpenMP support.

## Requirements
The implementation has been tested under Mac OSX and Ubuntu Linux. In order to build and use the system, the following are required:
* MATLAB (tested on R2011b and R2013a)
* gcc compiler (version 4.7 or greater)
* [OpenNI 1.5](http://www.openni.org/openni-sdk/openni-sdk-history-2/) (not compatible with OpenNI 2)

## Installation
Compile mex functions from an OS terminal with the following command: `MATLABDIR="/path/to/matlab" OPENNIDIR="/path/to/openni/include" make`. 
Where `"/path/to/matlab"` is the MATLAB root directory, and where `"/path/to/openni/include"` is the OpenNI headers directory (e.g. on Ubuntu it is typically "/usr/include/ni/").

(Note: For some linux distribution you may have linking problems with libstdc++, that will result in an error message when running the code: in this case, force matlab to compile using libstdc++ of your system and not its own version. One way to do so is to temporarily make the symbolic link in `MATLABDIR/sys/os/ARCH/libstdc++.so.*` to point to the system libstdc++ (typically under `/usr/lib`)).

Up to this stage you can use the system:
* live: with an OpenNI compatible RGB-D sensor (tested with Microsft Kinect and Asus Xtion Pro Live), directly connected to computer where the system is running.
* recorded videos: with ".oni" files, previously recorded with an RGB-D sensor.

If you want to integrate the system in a ROS environment, reading sensor data on the topics *"/camera/depth_registered/image_raw"* and *"/camera/rgb/image_raw"*, reading odometry data on the topic *"/odom"* and publishing the traked people on the topic *"/people"* with message type *"people_msg/People"*, you need to perform the following additional steps:
* copy the entire people_msg directory into your `ROS_PACKAGE_PATH`
* cd to `ROS_PACKAGE_PATH`
* build the package with `rosmake people_msg`

(Note: the ROS MATLAB BRIDGE used by our system needs an updated version of the "google-collect.jar" library. It's necessary to replace the file `MATLABDIR/java/jarext/google-collect.jar` with the file that can be downloaded [http://search.maven.org/remotecontent?filepath=com/google/guava/guava/13.0.1/guava-13.0.1.jar"](here), renaming it from "guava-13.0.1.jar" to "google-collect.jar" and copying it to `MATLABDIR/java/jarext/` directory).

## Usage
The files in the direcotry examples contain detailed explanations about the usage of the system with the various source type. To obtain more information about a particular function use the MATLAB command help.

# More information
Testing datasets, qualitative results and more details about the system are available at [bit.ly/perceivingpeople](http://bit.ly/perceivingpeople).



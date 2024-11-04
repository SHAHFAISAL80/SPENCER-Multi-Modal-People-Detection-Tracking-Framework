# SPENCER-Multi-Modal-People-Detection-Tracking-Framework
SPENCER Multi-Modal People Detection &amp; Tracking Framework

Features at a glance
Multi-modal detection: Multiple RGB-D & 2D laser detectors in one common framework.
People tracking: Efficient tracker based upon nearest-neighbor data association.
Social relations: Estimate spatial relations between people via coherent motion indicators.
Group tracking: Detection and tracking of groups of people based upon their social relations.
Robustness: Various extensions such as IMM, track initiation logic and high-recall detector input make the people tracker work relatively robustly even in very dynamic environments.
Real-time: Runs at 20-30 Hz on gaming laptops, tracker itself requires only ~10% of 1 CPU core.
Extensible and reusable: Well-structured ROS message types and clearly defined interfaces make it easy to integrate custom detection and tracking components.
Powerful visualization: A series of reusable RViz plugins that can be configured via mouse click, plus scripts for generating animated (2D) SVG files.
Evaluation tools: Metrics (CLEAR-MOT, OSPA) for evaluation of tracking performance.
ROS integration: All components are fully integrated with ROS and written in C++ or Python. ROS Kinetic, Melodic and Noetic are supported.
Motivation
The aim of the EU FP7 research project SPENCER was to develop algorithms for service robots that can guide groups of people through highly dynamic and crowded pedestrian environments, such as airports or shopping malls, while behaving in a socially compliant manner by e.g. not crossing in between families or couples. Exemplary situations that such a robot could encounter are visualized in below image on the right. To this end, robust and computationally efficient components for the perception of humans in the robot's surroundings needed to be developed.

SPENCER Use-Case    SPENCER Use-Case

Architecture
The following figure shows the real-time people and group detection and tracking pipeline developed in the context of the SPENCER project:

SPENCER Tracking Pipeline

The entire communication between different stages of our pipeline occurs via ROS messages which encourage reuse of our components in custom setups. The modular architecture allows for easy interchangeability of individual components in all stages of the pipeline.

Overview of available components
Message definitions
We provide a set of reusable ROS message type definitions, which we have successfully applied across various people detection and tracking scenarios, over different sensor modalities and tracking approaches. Most relevant messages can be found inside the spencer_tracking_msgs package.

We highly encourage reuse of these messages to benefit from our rich infrastructure of detection, tracking, filtering and visualization components! Existing detection and tracking algorithms can often easily be integrated by publishing additional messages in our format, or by writing a simple C++ or Python node that converts between message formats.

People detection
We have integrated the following person detection modules:

A reimplementation of a boosted 2D laser segment classifier, based upon the method by Arras et al. [3]
An RGB-D upper-body detector described more closely in [2], which slides a normalized depth template over ROIs in the depth image
A monocular-vision full-body HOG detector (groundHOG) [2], which based upon a given ground plane estimate determines the image corridor in which pedestrians can be expected. This detector is GPU-accelerated using CUDA. The contained cudaHOG library requires manual compilation and a recent CUDA SDK as well as an nVidia graphics card.
An RGB-D detector from PCL, which extracts candidate ROIs on a groundplane and then applies a linear HOG classifier [4]
Further external detectors which output geometry_msgs/PoseArray or people_msgs/PositionMeasurementArray messages can easily be integrated into our framework using the scripts from this package. Examples of such detectors include:

The laser-based leg detector from wg-perception, which might work better than our own laser detector if the sensor is located very close to the ground. See our wrapper package and leg_detectors.launch (replaces laser_detectors.launch).
The deep learning-based DROW and DR-SPAAM person detectors for 2D laser range data by Beyer et al. [5] and Jia et al. [6], which fuse information from multiple successive frames.
Multi-modal detection and fusion
For detection-to-detection fusion, we have implemented a series of nodelets which can be used to flexibly compose a fusion pipeline by means of roslaunch XML files. Details can be found in the spencer_detected_person_association package. The following figure shows an example configuration which was used during experiments in SPENCER:

Example multi-modal people tracking architecture

In case of detection-to-track fusion (currently not implemented), it is still advisable to publish a CompositeDetectedPerson message (via CompositeDetectedPersons) for each set of detections associated with a track, such that later on it is possible to go back to the original detections from a track, and lookup associated image bounding boxes etc. via the associated detection_id.

Person and group tracking
For person and group tracking, we currently provide exemplary code based upon a nearest-neighbor standard filter data association, which is robust enough in most use cases (especially if multi-modal detectors are being used). The people tracker has been enhanced with a track initiation logic and 4 different IMM-based motion models (constant velocity with low process noise, high process noise, coordinated turn and Brownian motion) to make tracking more robust.

The group tracker relies on social/spatial relations determined via the same coherent motion indicator features as described in [1].

Internally, we have already integrated more advanced methods, including a track-oriented multi-hypothesis person tracker [2], and a hypothesis-oriented multi-model multi-hypothesis person and group tracker [1]. These components use exactly the same ROS message definitions, however, they are not yet publicly available. The components available here were originally implemented as baseline methods for comparison.

Filtering of tracked persons & tracking metrics
The spencer_tracking_utils package contains a number of standalone ROS nodes that can filter an incoming set of TrackedPerson messages based upon different criteria, e.g. distance to the sensor/robot, visually confirmed tracks only, etc.

In spencer_tracking_metrics, we have wrapped publicly available implementations of different tracking metrics, such as CLEAR-MOT and OSPA, such that they are compatible with our message definitions. These are useful for evaluating tracking performance for a given groundtruth.

Import of old annotated logfiles
The srl_tracking_logfile_import package provides a Python script for importing old 2D laserscan logfiles in CARMEN format that have been annotated with groundtruth person tracks, such as these datasets.

Visualization
The srl_tracking_exporter package contains a useful Python script for rendering track trajectories, detections and robot odometry from a 2D top-down perspective as scalable vector graphics (SVGs). These can optionally be animated to visualize the evolution of one or multiple tracks over time.

One major highlight of our framework is a reusable and highly configurable set of custom RViz plugins for the visualization of:

Detected persons
Tracked persons (including occlusion state, associated detection ID, and covariance ellipses)
Social relations
Tracked groups
As an example, some features of the tracked persons display are:

Different visual styles: 3D bounding box, cylinder, animated human mesh
Coloring: 6 different color palettes
Display of velocity arrows
Visualization of the 99% covariance ellipse
Display of track IDs, status (matched, occluded), associated detection IDs
Configurable reduction of opacity when a track is occluded
Track history (trajectory) display as dots or lines
Configurable font sizes and line widths
All of the following screenshots have been generated using these plugins.

Example screenshots of our system in action
The following screenshots show our system in action, while playing back recorded data from a crowded airport environment:

Multi-modal people detection. In orange: 2D laser [3], cyan: upper-body RGB-D [2], yellow: monocular vision HOG [2], grey: fused detections (when using detection-to-detection fusion).
Example detection results

People tracking. In red: tracks visually confirmed by image-based detector
Example people tracking results

Group tracking via coherent motion indicator features, as described in [1]
Example group tracking results

Demo videos
Videos of the people detection and tracking system in action can be found on the SPENCER YouTube Channel:

Real-Time Multi-Modal People Tracking in a Crowded Airport Environment (RGB-D and 2D laser)
Single Person Guidance Scenario Prototype (2D laser only)
Group Guidance Scenario Prototype (2D laser only)
Runtime performance
On the SPENCER robot platform, which is equipped with front and rear RGB-D sensors (Asus Xtion Pro Live) and two SICK LMS500 laser scanners, we distributed the people and group detection & tracking system over two high-end gaming laptops (Intel Core i7-4700MQ, nVidia GeForce 765M). The detectors for the frontal sensors were executed on one laptop along with the detection-fusion pipeline. The detectors for the rear-facing sensors and the people and group tracking modules were executed on the second laptop. Both laptops were connected with each other and the rest of the platform via gigabit ethernet.

With this configuration, the components run in real-time at 20-25 Hz (with visualization off-loaded to a separate computer), even in crowded environments where more than 30 persons are concurrently visible.

Installation from L-CAS package repository
A packaged version of the entire framework for ROS Kinetic (Ubuntu 16.04 Xenial) and ROS Melodic (Ubuntu 18.04 Bionic) is kindly provided by the Lincoln Research Centre for Autonomous Systems (L-CAS) and built by their continuous integration system. You must first add their package repository to your apt/sources.list as described here. Then, install the framework via

sudo apt-get install ros-${ROS_DISTRO}-spencer-people-tracking-full
Note that the groundHOG detector in the packaged version is non-functional.

Building from source
In this paragraph, we describe how to build the framework from source code. This is useful, for example, if you want to conduct larger changes in the supplied launch files. You can either follow this step-by-step guide, or use one of the exemplary Docker files to automate these steps and isolate the build process from your existing ROS installation.

The people and group detection and tracking framework has been tested on Ubuntu 16.04 (ROS Kinetic), Ubuntu 18.04 (ROS Melodic) and Ubuntu 20.04 (ROS Noetic). For more information on the Robot Operating System (ROS), please refer to ros.org.

NOTE: The entire framework only works on 64-bit systems. On 32-bit systems, you will encounter Eigen-related alignment issues (failed assertions). See issue #1

1. Cloning the source code repository
First create an empty folder for a new catkin workspace and clone the GitHub repository into the src subfolder:

cd ~/Code
mkdir -p spencer-people-tracking-ws/src
cd spencer-people-tracking-ws/src
git clone https://github.com/spencer-project/spencer_people_tracking.git
git checkout $ROS_DISTRO
where in the last step, $ROS_DISTRO$ should refer to one of the available git branches specific to the particular ROS distribution (kinetic, melodic or noetic).

2. Installing required dependencies
Assuming you have ROS itself already installed, we recommend installing the required depencencies of our framework via:

rosdep update
rosdep install -r --from-paths . --ignore-src
3. Initializing the catkin workspace
Next, we suggest to use catkin (available via sudo apt-get install python-catkin-tools or python3-catkin-tools in ROS Noetic) to setup the workspace:

cd ..
catkin config --init --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
4. Building the ROS packages
Finally, build all packages via:

catkin build -c -s
5. Sourcing the ROS workspace
After building the workspace, source it via:

source devel/setup.bash
Special note on CUDA SDK for the groundHOG far-range detector
The cudaHOG library used by rwth_ground_hog requires an nVidia graphics card and an installed CUDA SDK (recommended version: 6.5). As installing CUDA (especially on laptops with Optimus/Bumblebee) and compiling the library is not straightforward, detailled installation instructions are provided here. Once these instructions have been followed, the rwth_ground_hog package needs to be rebuilt using catkin. If no CUDA SDK is installed, the ROS package will still compile, but it will not provide any functionality. As this detector is slightly outdated, we recommend to ignore this unless really needed. We hope to incorporate a more recent far-range person detector based upon a modern deep learning framework in the future.

Quick start tutorial
The following demo and three tutorials help you to easily get started using our framework.

Demo: Multimodal tracking in RGB-D and 2D laser from a bagfile
A short exemplary bagfile with 2D laser and RGB-D sensor data to test our framework can be downloaded by running

rosrun spencer_people_tracking_launch download_example_bagfiles.sh
Then, you can launch

roslaunch spencer_people_tracking_launch tracking_on_bagfile.launch
which will start playing back a bagfile (as soon as you unpause by pressing SPACE) and run Rviz for visualization. If everything went well, you should see detections from the upper-body detector, the 2D laser detector, and resulting person tracks.

Using the PCL people detector instead of the upper-body detector
As an alternative to the depth template-based upper-body detector, you can choose to use our slightly modified version of the person detector from the Point Cloud Library. This detector first performs a Euclidean clustering and head subcluster extraction, before validating the candidate ROIs using a HOG SVM. To do so, pass use_pcl_detector:=true to the launch file.

Enabling the groundHOG detector
If you have compiled the cudaHOG library (see description further above), you can optionally enable the groundHOG detector by passing use_hog_detector:=true to the launch file. The detection-to-detection fusion pipeline will then automatically fuse the detections from both detectors.

Tutorial 1: People / group tracking and visualization with a single RGB-D sensor
This is the easiest way to get started using just a single RGB-D sensor connected locally to your computer. Place your Asus Xtion Pro Live sensor horizontally on a flat surface, and connect it to your computer (or play the example bagfile linked in a section further below). Then run the following launch file from your people tracking workspace (make sure that you have sourced it, e.g. source devel/setup.bash):

roslaunch spencer_people_tracking_launch tracking_single_rgbd_sensor.launch height_above_ground:=1.6
This will do the following:

Start the OpenNi2 drivers (for Asus Xtion Pro) and publish RGB-D point clouds in the /spencer/sensors/rgbd_front_top/ camera namespace
Run an upper-body RGB-D detector, assuming a horizontal ground plane at 1.6 meters below the sensor. Other heights may work as well, but the detector has been trained at approximately this height.
Run a simple detection-to-detection fusion pipeline
Run the srl_nearest_neighbor_tracker, which will subscribe to /spencer/perception/detected_persons and publish tracks at /spencer/perception/tracked_persons
Run RViz with a predefined configuration, which shows the point cloud, the sensor's view frustum, and detected and tracked persons (using our custom RViz plugins).
Using MS Kinect v1
The original MS Kinect v1 sensor does not support OpenNi2. In this case, append use_openni1:=true to the above command-line of the launch file to fall back to OpenNi1.

Troubleshooting
If you cannot see any detection bounding boxes, first check if the point cloud is displayed properly in RViz. If not, there is probably a problem with your RGB-D sensor (USB or OpenNi issues).

Tutorial 2: Tracking with front and rear laser + RGB-D sensors
To try out a sensor configuration similar to the SPENCER robot platform, run:

roslaunch spencer_people_tracking_launch tracking_on_robot.launch
This assumes the RGB-D sensors mounted horizontally at about 1.6m above ground, and sensor data to be published on the following topics:

/spencer/sensors/laser_front/echo0  [sensor_msgs/LaserScan]
/spencer/sensors/laser_rear/echo0   [sensor_msgs/LaserScan]
/spencer/sensors/rgbd_front_top/{rgb/image_raw,depth/image_rect}  [sensor_msgs/Image]
/spencer/sensors/rgbd_front_top/{rgb/camera_info} [sensor_msgs/CameraInfo]
/spencer/sensors/rgbd_rear_top/{rgb/image_raw,depth/image_rect}   [sensor_msgs/Image]
/spencer/sensors/rgbd_rear_top/{rgb/camera_info}  [sensor_msgs/CameraInfo]
The launch file starts a pipeline similar to that from tutorial 1 (above), but includes a second set of RGB-D detectors for the rear sensor, as well as person detectors for the two 2D laser scanners. Sensor drivers which publish the RGB-D and laser data listed above are not started automatically by this launch file. Also, you manually have to start Rviz.

Using a subset of these sensors
Note that the fusion pipeline reconfigures automatically if only a subset of the person detectors is running. If e.g. you don't have a rear RGB-D sensor, just comment out the line which includes rear_rgbd_detectors.launch in tracking_on_robot.launch.

Tutorial 3: Fully customized sensor configuration
Start your own launch files for starting person detectors, or use a combination of the launch files we provide in spencer_people_tracking_launch/launch/detectors. You may have to remap input and output topics as needed.
Create a copy of detection_to_detection_fusion_pipeline.launch and its children, such as fuse_lasers_and_rgbd.launch, in spencer_detected_person_association. Based upon the provided example, create your own pipeline that step-by-step fuses detections from different detectors. For more information, see the corresponding package.
Create a copy of the freiburg_people_tracking.launch file in spencer_people_tracking_launch. Adjust it to refer to your own fusion launch file created in step 2.
Start your copy of freiburg_people_tracking.launch.
If needed, start group tracking via roslaunch spencer_people_tracking_launch group_tracking.launch.
Multi-modal datasets for evaluation
The multi-modal "Motion Capture" sequence from our ICRA 2016 paper is available upon request to let you evaluate your own detection / tracking algorithms on our dataset. For a fair comparison, please use the CLEAR-MOT metrics implementation contained in this repository, if possible. The raw data from the airport sequence cannot be shared for privacy reasons, though we might provide the extracted detections at a later point.

Credits & How to cite
The software in this repository is maintained by:

Timm Linder, formerly at the Social Robotics Lab, Albert-Ludwigs-Universität Freiburg
Stefan Breuers, formerly at the Computer Vision Group, RWTH Aachen University
Credits of the different ROS packages go to the particular authors listed in the respective README.md and package.xml files. See also the AUTHORS.md file for a list of further contributors.

This work has been supported by the European Commission under contract number FP7-ICT-600877 (SPENCER), and has received additional funding from the European Union's Horizon 2020 research and innovation programme under grant agreement No 732737 (ILIAD).

If you use the software contained in this repository for your research, please cite the following publication:

On Multi-Modal People Tracking from Mobile Platforms in Very Crowded and Dynamic Environments
Linder, T., Breuers, S., Leibe, B., Arras, K.O.
IEEE International Conference on Robotics and Automation (ICRA) 2016

also optionally:

People Detection, Tracking and Visualization using ROS on a Mobile Service Robot
Linder, T. and Arras, K.O.
Robot Operating System (ROS): The Complete Reference (Vol. 1).
Springer Studies in Systems, Decision and Control, 2016

The framework is further described in the PhD thesis:

Multi-Modal Human Detection, Tracking and Analysis for Robots in Crowded Environments Linder, T. PhD thesis, Technical Faculty, University of Freiburg, 2020

License & disclaimer
This software is a research prototype. It is not ready for production use. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).

Most of the ROS packages in this repository are released under a BSD license. For details, however, please check the individual ROS packages (package.xml or LICENSE file) as there are some exceptions.

References
[1] Linder T. and Arras K.O. Multi-Model Hypothesis Tracking of Groups of People in RGB-D Data. IEEE Int. Conference on Information Fusion (FUSION'14), Salamanca, Spain, 2014.

[2] Jafari O. Hosseini and Mitzel D. and Leibe B.. Real-Time RGB-D based People Detection and Tracking for Mobile Robots and Head-Worn Cameras. IEEE International Conference on Robotics and Automation (ICRA'14), 2014.

[3] Arras K.O. and Martinez Mozos O. and Burgard W.. Using Boosted Features for the Detection of People in 2D Range Data. IEEE International Conference on Robotics and Automation (ICRA'07), Rome, Italy, 2007.

[4] Munaro M. and Menegatti E. Fast RGB-D people tracking for service robots. In Autonomous Robots, Volume 37 Issue 3, pp. 227-242, Springer, 2014.

[5] Beyer L. and Hermans A. and Linder T. and Arras K. O. and Leibe B. Deep Person Detection in 2D Range Data. IEEE Robotics and Automation Letters 3(3), pp. 2726-2733, 2018.

[6] Jia D. and Hermans A. and Leibe B. DR-SPAAM: A Spatial-Attention and Auto-regressive Model for Person Detection in 2D Range Data. IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS'20), 2020.

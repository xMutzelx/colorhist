# Colorhist </br>
This repository contains a new post processing pipeline for ORK (Object Recognition Kitchen).
Colorhist provides a support for color detection and comparision of color histograms. </br>
</br>
If you want to use this pipeline, you have to take care of some things.</br>
<b>Installation tutorial:</b>

<b>create catkin_ws</b> </br>
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \ </br>
    mkdir -p /root/catkin_ws/src && \ </br>
    cd /root/catkin_ws/src && \ </br>
    catkin_init_workspace && \ </br>
    cd /root/catkin_ws && \ </br>
    catkin_make && \ </br>
    echo 'source /root/catkin_ws/devel/setup.bash' > /root/.bashrc</br></br>

<b>install ork dependencies</b> </br>
RUN apt-get -y install libopenni-dev \ </br>
    ros-$ROS_DISTRO-ecto* \ </br> 
    ros-$ROS_DISTRO-opencv-candidate \ </br>
    ros-$ROS_DISTRO-moveit-msgs \ </br>
    ros-$ROS_DISTRO-openni* && \ </br>
    source /root/catkin_ws/devel/setup.bash </br></br>

<b>clone ork gitlab repos</b> </br>
RUN cd /root/catkin_ws/src && \ </br>
    git clone http://github.com/wg-perception/object_recognition_core && \ </br>
    git clone http://github.com/wg-perception/capture && \ </br>
    git clone http://github.com/wg-perception/reconstruction && \ </br>
    git clone https://github.com/xMutzelx/ork_renderer.git && \ </br>
    git clone http://github.com/wg-perception/tabletop && \ </br>
    git clone https://github.com/xMutzelx/tod.git && \ </br>
    git clone http://github.com/wg-perception/transparent_objects && \ </br>
    git clone http://github.com/wg-perception/object_recognition_msgs && \ </br>
    git clone http://github.com/wg-perception/object_recognition_ros && \ </br>
    git clone http://github.com/wg-perception/object_recognition_ros_visualization && \ </br>
    git clone https://github.com/xMutzelx/ork_tutorials.git </br>

<b>change package versions</b> </br>
RUN cd /root/catkin_ws/src/capture && \ </br>
    git checkout 0.3.1 && \ </br>
    cd /root/catkin_ws/src/object_recognition_core && \ </br>
    git checkout fb3b3df && \ </br>
    cd /root/catkin_ws/src/object_recognition_ros_visualization && \ </br>
    git checkout f072ccf && \ </br>
    cd /root/catkin_ws/src/ork_renderer && \ </br>
    git checkout glut_fix && \ </br>
    cd /root/catkin_ws/src/reconstruction && \ </br>
    git checkout 8adb948 && \ </br>
    cd /root/catkin_ws/src/tabletop && \ </br>
    git checkout 7d49e3e && \ </br> 
    cd /root/catkin_ws/src/tod && \ </br>
    git checkout kinectv2_refactoring && \ </br>
    cd /root/catkin_ws/src/transparent_objects && \ </br>
    git checkout 75d7663 </br></br>

<b>install ros dependencies</b> </br>
RUN cd /root/catkin_ws && \ </br>
    apt-get update && apt-get install -y libvlccore-dev python-apt && \ </br>
    rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y </br></br>

<b>build all this shit</b> </br>
RUN cd /root/catkin_ws && \ </br>
    source devel/setup.bash && \ </br>
    catkin_make -j4 </br></br>

<b>build linemod</b> </br> 
RUN cd /root/catkin_ws/src && \ </br>
    git clone https://github.com/xMutzelx/linemod.git && \ </br>
    cd /root/catkin_ws/src/linemod && \ </br> 
    git checkout colorhist_ready && \ </br>
    cd /root/catkin_ws && \ </br>
    rosdep install --from-paths src -r -y -i && \ </br>
    source devel/setup.bash && \ </br>
    catkin_make -j4 </br></br>

<b>build colorhist</b> </br> 
RUN cd /root/catkin_ws/src && \ </br>
    git clone https://github.com/xMutzelx/colorhist.git && \ </br>
    cd /root/catkin_ws && \ </br>
    rosdep install --from-paths src -r -y -i && \ </br>
    source devel/setup.bash && \ </br>
    catkin_make -j4 </br></br>
    
I will soon publish an ready to use docker container. Link will follow. </br></br>

Training: rosrun object_recognition_core training -c `rospack find object_recognition_colorhist`/conf/training.ork --visualize </br>
The trainer takes the captured color pixels and creates a color histogram. </br>
Detection: rosrun object_recognition_core detection -c  `rospack find object_recognition_colorhist`/conf/detection.ros.ork </br>
The output from Linemod (Object ID, segmented Image) gets analized and compared with the relevant models in the database. </br>
Now you can detect different sorts of one object (Pringles red, green, purple, ...).</br>

<b>Caution: Everything is under construction! It might not work in the current state. A new release will follow soon.</b></br>

<b>Tutorial</b> (I asume that you have basic knowledge about ORK and Linemod):</br>
1) Add some objects to your database. Important: You can only use trained objects, not modeled objects (example: blender).</br>

2) You have to define one "parent" model in your database</br>
- You have to add the field "Skip" in your database document. Path: by_object_id_and_mesh: 
  - 0: train it in Linemod (parent model, example: Pringles purple)
  - 1: do not train it in Linemod (child model, example: Pringles green, yellow, ...) </br>

3) Train Linemod: rosrun object_recognition_core training -c `rospack find object_recognition_linemod`/conf/training.ork --visualize </br></br>
4) Train Colorhist: rosrun object_recognition_core training -c `rospack find object_recognition_colorhist`/conf/training.ork --visualize </br></br>
5) You have to define the variations of an object by hand </br>
- Add the field "Variations" in all parent database documents (example: Pringles purple). Path:by_object_id_and_ColorHist:
  - Add all child ID's (example: Pringels yellow, red, ...) into this field, and seperate them with ";"
  Caution: Don't use blanks and add an ";" at the end. (example: "7d7583e438b42e673d3c6a358e00009f;7d7583e438b42e673d3c6a358e03a12a;") </br>
  
6) Test: 
- Start your camera: roslaunch openni_launch openni.launch
- Start depth regestration: rosrun rqt_reconfigure rqt_reconfigure (camera->driver->depth_registration) 
- Start your detection: rosrun object_recognition_core detection -c  `rospack find object_recognition_colorhist`/conf/detection.ros.ork
- You can visualize the results in RVIZ: rosrun rviz rviz </br>

The code for Kinectv2 is already implemented, but it is commented. I will soon add a flag for an easy switch between both cameras.

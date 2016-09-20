<b>Colorhist</b> </br>
This repository contains a new post processing pipeline for ORK (Object Recognition Kitchen).
Colorhist provides a support for color detection and comparision of color histograms. </br>
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

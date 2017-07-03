# icub-gazebo-skin

![icub-skin](http://www.ausy.tu-darmstadt.de/uploads/Research/SkinGazeboYarpPlugin/skin_torso.png)

This is the Gazebo-Yarp Plugin to simulate the iCub Skin. 
This code was developed as part of a student project and its final report is available [here](http://www.ausy.tu-darmstadt.de/uploads/Team/RobertoCalandra/Moritz_Jan_Towards_Balancing_with_the_iCub.pdf) (also contain few technical information).

Gazebo Yarp Plugin – Skin To reach the goal of improving the balancing of the iCub with the help of skin data we first implemented a skin model in the simulator. In order to do so we created a gazebo-yarp-plugin which insures the communication between Gazebo and Yarp. We have implemented a skin model in Gazebo, used a Yarp plugin for communications and put some logic into a Gazebo plugin.

We implemented the following files: - Skin.hh and Skin.cc: first part of the gazebo yarp plugin – i.e register plugin - SkinDriver.h and SkinDriver.cc: second part of the gazebo yarp plugin – i.e compute sforces and generates output - in icub_skin_version you can find the modified iCub models as sdf file - don't forget to edit the Cmake Files if you want to change/compile the plugin - java scripts to generate the sdf files

Developed by Geukes, J., Nakatenus, M. and Calandra, R. 

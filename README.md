Coleman University & Electronic Networked Vehicle Institute
Robosub competition files 2017
</br>
</br>
The software runs on an NIVIDIA Jetson TX2 with Jetpack 3.0 (Linux 4 Tegra L4T 27.1), which is ARM architecture.
The TX2 is interfaced over USB with a Pixhawk flight controller with Ardusub 3.5.1 firmware, which is interfaced with 6 30Amp ESCs that run to 6 DC motors, and a depth pressure sensor. The two boards communicate using the <a href="http://qgroundcontrol.org/mavlink/start">MAVLink</a> protocol.
</br>
</br>
</br>
<strong>Environment Setup</strong>
</br>
1. ROS Indigo    http://wiki.ros.org/indigo/Installation/UbuntuARM
2. MAVROS        https://dev.px4.io/en/ros/mavros_installation.html
3. TensorFlow    https://github.com/jetsonhacks/installTensorFlowTX2  
    (This info may be out of date, try to work through errors)
</br>
<strong>Building</strong>
</br>
<code>cd ~/catkin_ws</code> (or wherever you have your catkin workspace)
</br>
<code>git clone https://github.com/ENVI-UUV/RobosubROSAutoPilot.git</code>
</br>
<code>catkin_make</code> or <code>catkin_build</code>

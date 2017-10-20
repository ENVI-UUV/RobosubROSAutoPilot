Coleman University & Electronic Networked Vehicle Institute
Robosub competition files 2017
<br/>
<br/>
The software runs on an NIVIDIA Jetson TX2 with Jetpack 3.0 (Linux 4 Tegra L4T 27.1), which is ARM architecture.
The TX2 is interfaced over USB with a Pixhawk flight controller with Ardusub 3.5.1 firmware, which is interfaced with 6 30Amp ESCs that run to 6 DC motors, and a depth pressure sensor. The two boards communicate using the <a href="http://qgroundcontrol.org/mavlink/start">MAVLink</a> protocol.
<br/>
<br/>
<br/>
<strong>Environment Setup</strong>
<br>
<table>
  <tr>
    <td>1.</td>
    <td>ROS Indigo</td>
    <td>http://wiki.ros.org/indigo/Installation/UbuntuARM</td>
  </tr>
  <tr>
    <td>2.</td>
    <td>MAVROS</td>
    <td>https://dev.px4.io/en/ros/mavros_installation.html</td>
  </tr>
  <tr>
    <td>3.</td>
    <td>TensorFlow</td>
    <td>https://github.com/jetsonhacks/installTensorFlowTX2<br/>(This info may be out of date, try to work through errors)</td>
  </tr>
</table>
<br/>
<strong>Building</strong>
<br/>
<code>cd ~/catkin_ws</code>
<br/>
<code>git clone https://github.com/ENVI-UUV/RobosubROSAutoPilot.git</code>
<code>catkin_make</code>
<br/>
<br/>
<strong>Running</strong>
<br/>
<code>roslaunch RobosubROSAutoPilot robosub.launch</code>

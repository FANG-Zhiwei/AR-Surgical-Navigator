# ARSugicalNavigator

Introduction
------------

Guideline for ROS application
-----------------------------
**1. Installation <p/>**

   Download the ros2_ws file or extract the src.tar.gz into your workspace, then use <code>colcon build</code> to build the file. Before that, make sure your linux has installed ros2 humble.

**2. Commands <p/>**

   Then, open the terminal and use <code>source/install/setup.bash</code> to source your workspace in order to run the follwing command:
   
    ros2 run ros_tcp_endpoint default_server_endpoint --ros-args -p ROS_IP:=192.168.86.129
    
   The above command turns on the ros tcp endpoint for connection between Unity and ROS, for the ROS_IP, since every device is of different IP, you must search for it by typing <code>hostname --I</code>, and copy the IP address to replace the one stated on the above command.<p/>
      
   After running the command, you may simply run the app in Hololens or run the Unity play scene in computer for simulation and debug. Note that the two devices (Linux computer & Hololens/Windows computer) should remain in same network.</p>

   Then, you may open the node for publishing the MRI image data to Unity, the node is also subscribing for the needle motion data from Unity in order to adjust the MRI image display, by using the following command:
   
   <code>ros2 run mri_tomo_display mri_image</code><p/>
   
   The above command execute to display the OCT video tape on the Unity play scene.<p/>
   
   <code>ros2 run mri_tomo_display oct_image</code><p/>
   

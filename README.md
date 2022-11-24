# robo-arm-ROS2
<h2>Robotic Arm ROS 2 packages for book sorting</h2>
  <body>
    Prerequisite:<br>
    MoveIt2 Foxy distro<br>
    https://moveit.picknik.ai/foxy/index.html<br><br>
  
  The following will install from Debian any package dependencies not already in your workspace.<br>
  This is the step that will install MoveIt and all of its dependencies:<br>
    <code>sudo apt update && rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y</code><br>
    
  Create a new workspace and clone in the src folder<br>
    <code>cd ~/robo_arm_ws</code>
    <br>

  To launch<br>
    <code>ros2 launch robo-arm-bringup demo.launch.py</code><br>
  </body>
  
  Upon launching<br>
    In the RViz, choose "Add" and choose Motion Planning<br>

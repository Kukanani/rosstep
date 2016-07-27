# ROSSTEP

ROSSTEP is a system for sequentially running roslaunch, rosnode, and bash scripts automatically,
for use in Robot Operating System (ROS) applications. The system consists of YAML files which
define actions and conditions. A python file parses the code and runs actions sequentially using
the sys and subprocess python modules. Between actions, it uses various ROS-based code to check
conditions required to proceed, and only moves on to the next action when all the necessary
conditions have been met.  

rosstep-creator is a QT application designed to create the YAML files required for ROSSTEP. It
has a nearly one-to-one mapping from interface elements to YAML output, and serves as a convenient
GUI for working with the ROSSTEP system.

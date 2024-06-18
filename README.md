# pepper-CPG-ros
transfer for ros stuff, adapted from pepper-CPG-main


not a functional ros repo, make the workspace first then put those files in, see requirments below (to do)




Commandes récurrentes: ########################################################################



cd ros2_iron_ws
cd home/pepper/ros2_iron_ws/src   DANS SRC

apres avoir tout vidé:
ros2 pkg create --build-type ament_python my_hand_tracking_pkg
mettre les bons xml et setup
mettre hand node dans le double doss


ensute   DANs ROSIRONWS
cd ros2_iron_ws


colcon build

sourcage :

source /opt/ros/iron/setup.bash
#source ~/ros2_iron_ws /install/setup.bash
source install/setup.bash


#essai cht de repo : 
cd src/my_hand_tracking_pkg
cd ~/ros2_iron_ws/src


ros2 run my_hand_tracking_pkg hands_tracking_node


ros2 run my_hand_tracking_pkg hands_tracking_node
#### pourafficher, normalement, my hand tracking pkg
ros2 pkg list | grep my_hand_tracking_pkg

##################################################
##################################################
 a chaque cht de code on rebuild le workspace
 
 cd ~/ros2_iron_ws
 
 
 
rm -rf build/ install/ log/
colcon build
source /opt/ros/iron/setup.bash
source install/setup.bash
ros2 run my_hand_tracking_pkg hands_tracking_node


#######################################################

assurer que les dépendencies python sont prise en compte après les modifs du xml:

colcon build --packages-select my_hand_tracking_pkg
source install/setup.bash


#######################################################









# Package building dependencies
- executive_smach
- mummer_integration_msgs
- dialogue_as

# Execution dependencies
## Bring up Pepper
```
roslaunch perspectives bringup_pepper.launch
```
**Services**
  - /naoqi_driver/robot_posture/go_to_posture

## Perspectives and control modules
```
uwds start
load_ws
roslaunch perspectives mummer.launch
```
**Services**
  - /speech_wrapper/speak_to
  - /uwds_ros_bridge/has_mesh
  - /multimodal_human_monitor/monitor_humans
  - /uwds_ros_bridge/start_fact
  - /uwds_ros_bridge/end_fact
  - /multimodal_human_monitor/find_alternate_id
  - /deictic_gestures/look_at
  - /deictic_gestures/point_at
  
**Topics**
  - /base/current_facts
  - /head_manager/head_coordination_signals

## Ontology modules
```
roslaunch perspectives ontology.launch
```
**Services**
  - /semantic_route_description/get_route_region
  - /route_description/get_region_route
  - /ontoloGenius/individual

## Pointing planner
```
docker run --net=host -e ROS_MASTER_URI=${ROS_MASTER_URI} -v ${current_dir}/move4d:/move4d move4d_ros
```
**Action server**
  - pointing_planner/PointingPlanner

## Navigation
From the docker repository
```
 docker run --net=host -e ROS_MASTER_URI=${ROS_MASTER_URI} nav:nav 
```
**Action server**
  - m_move_to

## Dialogue
```
python -m abcdk.AbcdkSoundReceiver.AbcdkSoundReceiver --qi-url mummer-eth0.laas.fr
roslaunch dialogue LAAS_dialogue.launch # Control the dialogue module on Pepper
rosrun dialogue_as dialogue_as # Dialogue action server to get keywords from a speaker
```
**Action server**
  - dialogue_as

## Perception (only window n°2) 
### /!\ be careful to your .bashrc configuration
```
rosrun person_manager start_perception.sh -c config_openpose.ini
```

# The state machine
Launch the action server
```
roslaunch guiding_as guiding_as.launch
```
To start the state machine by sending a goal = a target + a human
```
rosrun guiding_as guiding_client.py
```
To visualize the state machine and its states
```
rosrun smach_viewer smach_viewer.py
```

# Sources
https://github.com/LAAS-HRI/perspectives

https://github.com/LAAS-HRI/semantic_route_description

https://github.com/LAAS-HRI/mummer_integration/tree/master/navigation

https://github.com/LAAS-HRI/move4d_ros_docker

# State machine diagrams
## Global diagram
![state machine overview](img/global_graph.svg)
## Diagram of "Show" state (sub-state machine)
![state machine overview](img/show.svg)


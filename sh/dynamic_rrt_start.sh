#!/bin/sh
#Move to the start point of RRT Planning 
rostopic pub -1 /joint1_position_controller/command std_msgs/Float64 "0.8407" &
rostopic pub -1 /joint2_position_controller/command std_msgs/Float64 "1.2096" &
rostopic pub -1 /joint3_position_controller/command std_msgs/Float64 "0.0" &
rostopic pub -1 /joint4_position_controller/command std_msgs/Float64 "0.635" &
rostopic pub -1 /joint5_position_controller/command std_msgs/Float64 "0.0" &
rostopic pub -1 /joint6_position_controller/command std_msgs/Float64 "0.0" &
rostopic pub -1 /joint7_position_controller/command std_msgs/Float64 "0.0"	

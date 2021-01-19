#!/bin/bash

#save the directory that the script was run in
original_cd=$PWD

#source the ros setup script and then source the workspace setup script in order to use ros commands
source /opt/ros/kinetic/setup.bash
source $ROS_WORKSPACE/devel/setup.bash

# Determine which arguments correspond to each variable based on the the total amount of arguments entered
if [[ $# -eq 4 ]]; then
    level=$1
    sim=$2
    gui="true"
    robot=$3  
    iterations=$4
    obstacle_bot="false"
elif [[ $# -eq 5 ]]; then
    level=$1
    sim=$2
    gui=$3
    robot=$4   
    iterations=$5
    obstacle_bot="false"
elif [[ $# -eq 6 ]]; then
    level=$1
    sim=$2
    gui=$3
    robot=$4   
    iterations=$5
    obstacle_bot=$6 
fi

# Launch a world in Gazebo
echo "Setting up the $robot robot in the $level environment"
roslaunch uml_3d_race "$level".launch sim:="$sim" gui:="$gui" navigate:=true robot:="$robot" obstacle_bot:="$obstacle_bot" &
pid1=$!
sleep 15s

echo "Starting loggers"
. "$ROS_WORKSPACE"/src/uml_3d_race/scripts/log_sim_data.sh $level &
pid2=$!
sleep 10s


echo "Starting test"
roslaunch uml_3d_race race.launch iterations:="$iterations"

#once the script raches this point, the race launch file has finished running
echo "The test has finished, killing all of the test processes"

#first kill the loggers
kill -2 $pid2
wait $pid2

#now kill the test setup launch file
kill -2 $pid1
wait $pid1

#finally cd to the original directory the script was run in
cd $original_cd
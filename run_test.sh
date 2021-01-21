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
echo -----------------------------------------------------------
echo
echo "Setting up the $robot robot in the $level environment"
echo
echo -----------------------------------------------------------
roslaunch uml_3d_race "$level".launch sim:="$sim" gui:="$gui" navigate:=true robot:="$robot" obstacle_bot:="$obstacle_bot" &
pid1=$!

sleep 12s

echo -----------------------------------------------------------
echo
echo "Starting loggers"
echo
echo -----------------------------------------------------------
#cd to the logs folder inside of resources in the uml_3d_race package
roscd uml_3d_race
cd resources/logs

#create a folder to put the logs into and name the folder using the name of the map being used and the current time
name=$1_$(date +'%F_%T')
mkdir $name
cd $name

#create a folder for the geotiff maps to live in
mkdir geotiff_maps

#start the logging scripts
roslaunch uml_3d_race geotiff_writer.launch map_dir:=$ROS_WORKSPACE/src/uml_3d_race/resources/logs/$name/geotiff_maps &
pid2=$!
rostopic echo -p sim_log > sim_log.csv &
pid3=$!

sleep 5s

echo -----------------------------------------------------------
echo
echo "Starting test"
echo
echo -----------------------------------------------------------
roslaunch uml_3d_race race.launch iterations:="$iterations"
pid4=$!

#this will kill the script if any errors occur during the test
trap "kill -2 $pid4; kill -2 $pid3; kill -2 $pid2; kill -2 $pid1; wait; trap - INT TERM ERR; cd $original_cd" INT TERM ERR

#once the script raches this point, the race launch file has finished running

echo -----------------------------------------------------------
echo
echo "The test has finished, killing all of the test processes"
echo
echo -----------------------------------------------------------

#kill all of the background processes
kill -2 $pid3
kill -2 $pid2
kill -2 $pid1
wait

#Reset trap
trap - INT TERM ERR

#finally cd to the original directory the script was run in
cd $original_cd
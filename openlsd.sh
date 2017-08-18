#!/bin/bash
#Open up the nessesary lsd_slam nodes.

open_example() {
  #Open the example provided by the creaters
  xterm  -e 'roscore' &
  xterm  -e 'source ~/RCS_ROS_WS/devel/setup.bash ; rosrun lsd_slam_viewer viewer' &
  xterm  -e 'source ~/RCS_ROS_WS/devel/setup.bash ; rosrun lsd_slam_core live_slam image:=/image_raw camera_info:=/camera_info' &
  rosbag play ~/RCS_ROS_WS/src/lsd_slam/bag_files/LSD_room.bag
}

open_images(){
  #Open your own set of images
  if [ "$2" = "0" ] || [ "$3" = "0" ] || [ "$4" = "0" ]
  then
    show_usage
  else
    xterm  -e 'roscore' &
    xterm  -e 'source ~/RCS_ROS_WS/devel/setup.bash ; rosrun lsd_slam_viewer viewer' &
    source ~/RCS_ROS_WS/devel/setup.bash
    echo "$2"
    #rosrun lsd_slam_core dataset_slam _files:=$2 _hz:=$3 _calib:=$4
  fi
}

show_usage(){
  #The user has done something wrong, show them how to use the script
  echo -e "Usage: \n \t Open example lsd_slam: \t ./openlsd.sh example \n \t Open lsd from images: \t ./openlsd.sh images <image folder> <fps>  <calibration file>"
}

if [ $# -eq 0 ]
  then
    open_example
elif [ $1 = 'example' ]
  then
    open_example
elif [ $1 = 'images' ]
  then
    open_images
else
    show_usage
fi

#!/bin/sh
# author: Radek Janečka
# email: gitjaneckar@seznam.cz
# file: setup_rosdep.sh

# checks if the user runs the script as root
if [ "$(id -u)" -ne 0 ]; then
    echo "This script must be run as root. Use: sudo $0"
    exit 1
fi

# variables
rosdep_keys_yaml=morve_rosdep_keys.yaml
rosdep_conf_location=/etc/ros/rosdep/sources.list.d
rosdep_list_file=42-custom_rosdep.list

HOME_DIR=/home/$SUDO_USER
rosdep_local_dir=$HOME_DIR/.rosdep/local
yaml_list_content=`echo "yaml file://$HOME_DIR/.rosdep/local/morve_rosdep_keys.yaml"`

# move file morve_rosdep_keys.yaml to home directory in new directory tree
mkdir -p $rosdep_local_dir
cp -f ./$rosdep_keys_yaml $rosdep_local_dir/$rosdep_keys_yaml

# create or overwrites $rosdep_list_file
sudo echo $yaml_list_content > $rosdep_conf_location/$rosdep_list_file

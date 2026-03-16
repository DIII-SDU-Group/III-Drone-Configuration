#!/bin/sh

set -e

SCRIPT_DIR=$(dirname $0)
CONFIG_DIR=$(cd $SCRIPT_DIR/../config && pwd)

$SCRIPT_DIR/update_installed_parameters.py $CONFIG_DIR/parameters/parameters.yaml $CONFIG_DIR/parameters

# Get target config dir from the first argument
if [ -n "$1" ]; then
    target_config_dir=$1
else
    echo "No target config directory specified. Exiting."
    exit 1
fi

mkdir -p $target_config_dir/iii_drone/parameters/

if [ ! -f $target_config_dir/iii_drone/parameters/parameter_manifest.yaml ]; then
    cp $CONFIG_DIR/parameters/parameter_manifest.yaml $target_config_dir/iii_drone/parameters/parameter_manifest.yaml
fi

$SCRIPT_DIR/update_installed_parameters.py $CONFIG_DIR/parameters/parameters.yaml $target_config_dir/iii_drone/parameters/

if [ ! -f $target_config_dir/iii_drone/ros_params_real.yaml ]; then
    cp -f $CONFIG_DIR/ros_params_real.yaml $target_config_dir/iii_drone/ros_params_real.yaml
fi

if [ ! -f $target_config_dir/iii_drone/ros_params_sim.yaml ]; then
    cp -f $CONFIG_DIR/ros_params_sim.yaml $target_config_dir/iii_drone/ros_params_sim.yaml
fi

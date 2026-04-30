#!/bin/sh

set -e

SCRIPT_DIR=$(dirname $0)
CONFIG_DIR=$(cd $SCRIPT_DIR/../config && pwd)

$SCRIPT_DIR/update_installed_parameters.py $CONFIG_DIR/parameters/parameter_manifest.yaml $CONFIG_DIR/parameters

# Get target config dir from the first argument
if [ -n "$1" ]; then
    target_config_dir=$1
else
    echo "No target config directory specified. Exiting."
    exit 1
fi

mkdir -p $target_config_dir/iii_drone/parameters/
mkdir -p $target_config_dir/iii_drone/profiles/
mkdir -p $target_config_dir/iii_drone/parameter_sets/

if [ ! -f $target_config_dir/iii_drone/parameters/parameter_manifest.yaml ]; then
    cp $CONFIG_DIR/parameters/parameter_manifest.yaml $target_config_dir/iii_drone/parameters/parameter_manifest.yaml
fi

$SCRIPT_DIR/update_installed_parameters.py $CONFIG_DIR/parameters/parameter_manifest.yaml $target_config_dir/iii_drone/parameters/

cp -rn $CONFIG_DIR/profiles/. $target_config_dir/iii_drone/profiles/
cp -rn $CONFIG_DIR/parameter_sets/. $target_config_dir/iii_drone/parameter_sets/

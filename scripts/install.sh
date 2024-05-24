#!/bin/sh

set -x
set -e

SCRIPT_DIR=$(dirname $0)


mkdir -p ~/.config/iii_drone/parameters/

if [ ! -f ~/.config/iii_drone/parameters/parameters.yaml ]; then
    cp $SCRIPT_DIR/../config/parameters.yaml ~/.config/iii_drone/parameters/parameters.yaml
else
    $SCRIPT_DIR/update_installed_parameters.py $SCRIPT_DIR/../config/parameters.yaml ~/.config/iii_drone/parameters/
fi

cp -f $SCRIPT_DIR/../config/ros_params.yaml ~/.config/iii_drone/ros_params.yaml
rm -rf ~/.config/iii_drone/node_parameters 2> /dev/null
cp -rf $SCRIPT_DIR/../config/node_parameters ~/.config/iii_drone/
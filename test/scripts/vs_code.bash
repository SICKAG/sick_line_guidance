#!/bin/bash


if [ -f /opt/ros/noetic/setup.bash ] ; then 
    source /opt/ros/noetic/setup.bash
elif [ -f /opt/ros/foxy/setup.bash ] ; then 
    source /opt/ros/foxy/setup.bash
fi
pushd ../../../..
if [ -f ./install/setup.bash ] ; then 
    source ./install/setup.bash
fi

code ./sick_line_guidance_vscode.code-workspace
popd 

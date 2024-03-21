#!/bin/bash

# init ros environment
source /opt/ros/melodic/setup.bash
source ../../../../devel/setup.bash

# start edit resource-files
gedit ./run.bash ./runsimu.bash ../../../sick_line_guidance/launch/sick_canopen_simu.launch ../../../sick_line_guidance/launch/sick_line_guidance.launch ../../../sick_line_guidance/ols/sick_line_guidance_ols10.yaml ../../../sick_line_guidance/ols/sick_line_guidance_ols20.yaml ../../../sick_line_guidance/mls/sick_line_guidance_mls.yaml &

# start clion
# pushd ../../../sick_line_guidance
# ~/Public/clion-2018.3.3/bin/clion.sh &
pushd ../../../..
~/Public/clion-2018.3.3/bin/clion.sh ./src & 
popd


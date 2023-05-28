#!/bin/sh
pack_path=$(rospack find avisengine_environment)

#export the gazebo pathes
export GAZEBO_MODEL_PATH=$pack_path/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$pack_path:/usr/share/gazebo-11:/usr/share/gazebo-11:/usr/share/gazebo_models:$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=$pack_path/plugins:$GAZEBO_PLUGIN_PATH


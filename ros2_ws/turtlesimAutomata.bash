#!/bin/bash
set -e

# Defining workspace directory
WORKSPACE_DIR=~/ros2_ws

# Defining package source directory
PACKAGE_NAME=turtlesim_automata
PACKAGE_SRC_DIR=~/Downloads/$PACKAGE_NAME

# Ensuring the workspace directory exists
mkdir -p "$WORKSPACE_DIR/src"

# Checking if the package exists in the workspace
if [ ! -d "$WORKSPACE_DIR/src/$PACKAGE_NAME" ]; then
    if [ ! -d "$PACKAGE_SRC_DIR" ]; then
        echo "$PACKAGE_NAME package not found in the Downloads directory."
        exit 1
    fi
    echo "Copying $PACKAGE_NAME package to $WORKSPACE_DIR/src..."
    cp -r "$PACKAGE_SRC_DIR" "$WORKSPACE_DIR/src/"
else
    echo "$PACKAGE_NAME package already exists in $WORKSPACE_DIR/src."
fi

# Removing package from Downloads directory if it exists
if [ -d "$PACKAGE_SRC_DIR" ]; then
    rm -rf "$PACKAGE_SRC_DIR"
    echo "Package removed from Downloads directory."
fi

# Building the workspace
cd "$WORKSPACE_DIR"
source /opt/ros/humble/setup.bash
colcon build

# Sourcing the overlay
source install/setup.bash

# Runing the turtlesim_automata node in the background
ros2 run $PACKAGE_NAME turtlesim_automata_node &
AUTOMATA_PID=$!

# Waiting a few seconds to ensure the turtlesim_automata node is running
sleep 5

# Runing the turtlesim node in the foreground
ros2 run turtlesim turtlesim_node

# Waiting for the turtlesim_automata node to complete
wait $AUTOMATA_PID


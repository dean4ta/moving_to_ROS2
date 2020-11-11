#!/bin/bash

set -e

source /opt/ros/melodic/setup.bash
source /opt/ros/eloquent/setup.bash
source /root/ros_ws/install/setup.bash
/bin/bash
exec "$@"
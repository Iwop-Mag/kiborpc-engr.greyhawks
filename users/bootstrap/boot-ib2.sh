#!/bin/bash

CHECKPOINT_SOURCE_PATH=/tmp/share/logs/latest/cheat_sheet.json

source /opt/ros/melodic/setup.bash
source /users/catkin_ws/devel/setup.bash

# Wait the simulator to be ready
while [ "$(rosparam get /krpc/ready 2>&1)" != "true" ];
do
    sleep 2;
done

if [ ! -f ${CHECKPOINT_SOURCE_PATH} ]; then
    echo "checkpoint source file not found: ${CHECKPOINT_SOURCE_PATH}" >&2
    exit 1
fi

echo "Launch unified KRPC (API + user)"
bash $(dirname $0)/tail_log.sh &

# ROS logs land under the shared /tmp/share mount; run.sh extracts
# user_program_output.log from them once the simulation fully stops.
roslaunch krpc_user krpc_user.launch checkpoint_source_path:=${CHECKPOINT_SOURCE_PATH} 2>&1 > /dev/null
exit $?

#!/bin/bash

LOG_DIR="${ROS_LOG_DIR:-${HOME}/.ros/log/latest}"

while true;
do
    latest_log=$(ls -1t "${LOG_DIR}"/krpc_user-*.log 2>/dev/null | head -n 1)
    if [ -n "${latest_log}" ] && [ -f "${latest_log}" ]; then
        break
    fi
    sleep 1
done

tail -f "${latest_log}"

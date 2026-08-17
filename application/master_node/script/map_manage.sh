#!/bin/bash
## import scripts
current_path=$(cd `dirname $0`; pwd)
. ${current_path}/common_op.sh

launch_file="drobot_map_manager.launch"
find_pid_key="drobot_map_manager"

function look()
{
    look_proc ${find_pid_key}
}

function launch()
{
    if [[ ! -f ${DROBOT_LAUNCH_DIR}/${launch_file} ]]; then
        echo "launch file not found:${DROBOT_LAUNCH_DIR}/${launch_file}"
        return 1
    fi
    shutdown
    look_proc ${find_pid_key}
    if [[ ${DROBOT_RECENT_PROC_STOPED} == 1 ]]; then
        echo "launch ${launch_file} cmd"
        roslaunch ${DROBOT_LAUNCH_DIR}/${launch_file} &> ${DROBOT_LOG_DIR}/drobot_map_manager.log &
    else
        echo "${find_pid_key} is running, I will not launch it !"
    fi
    look
}

function shutdown()
{
    kill_proc ${find_pid_key}
    look
}

$@

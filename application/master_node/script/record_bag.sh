#!/bin/bash
## import scripts
current_path=$(cd `dirname $0`; pwd)
. ${current_path}/common_op.sh

launch_file="rosbag_record.py"
find_pid_key="rosbag_record.py"

function look()
{
    look_proc ${find_pid_key}
}

function launch()
{
    script_file=${DROBOT_LAUNCH_DIR}/${launch_file}
    if [[ ! -f ${script_file} ]]; then
        echo "launch file not found:${script_file}"
        return 1
    fi
    shutdown
    look_proc ${find_pid_key}
    if [[ ${DROBOT_RECENT_PROC_STOPED} == 1 ]]; then
        echo "launch ${script_file}"
        echo "args: ${1}"
        python ${script_file} ${DROBOT_BAG_DIR}/bag_file.bag &> ${DROBOT_LOG_DIR}/record_bag.log &
    else
        echo "${find_pid_key} is running, I will not launch it"
    fi
    look
}

function shutdown()
{
    kill_proc ${find_pid_key}
    look
}

$@

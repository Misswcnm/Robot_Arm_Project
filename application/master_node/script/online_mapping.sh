#!/bin/bash
## import scripts
current_path=$(cd `dirname $0`; pwd)
. ${current_path}/common_op.sh

launch_file="drobot_online_mapping.launch"
find_pid_key="drobot_online_mapping"

#return 0:process is running
#return 1:process is stoped
function look()
{
    look_proc ${find_pid_key}
    return $?
}

function launch()
{
    if [[ ! -f ${DROBOT_LAUNCH_DIR}/${launch_file} ]]; then
        echo "launch file not found:${DROBOT_LAUNCH_DIR}/${launch_file}"
        return 1
    fi
    look
    if [[ $? == 1 ]]; then
      echo -e "\e[1;34mExcute Launch [$launch_file] file... ...\e[0m"
      roslaunch ${DROBOT_LAUNCH_DIR}/${launch_file} &> ${DROBOT_LOG_DIR}/drobot_mapping.log &
      sleep 3
      look
      if [[ $? == 1 ]]; then 
          echo -e "[\e[1;31mFailed: launch ${launch_file}\e[0m]"
          return 1
      fi
    else 
      echo -e "\e[1;31mFailed: [$launch_file] is Running ... ...\e[0m"
    fi
}

function shutdown()
{
    kill_proc ${find_pid_key}
    timeout=0
    ret=0
    look
    ret=$?
    while [[ $ret == 0  &&  $timeout < 3  ]]; do
      ((timeout++))
      sleep 1
      look
      ret=$?
    done
    return $ret
}

$@

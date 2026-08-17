#!/bin/bash
## import scripts
current_path=$(cd `dirname $0`; pwd)
. ${current_path}/common_op.sh

launch_file="drobot_navigation.launch"
find_pid_key="drobot_navigation"
#0 means is running 1means is stops
function look()
{
    look_proc "amcl"
    amcl_status=$?
    look_proc "move_base"
    move_base_status=$?
    look_proc "path_reco"
    path_reco_status=$?
    look_proc "map_manage"
    map_manage_status=$?
    look_proc ${find_pid_key}
    drobot_navigation_status=$?

    if [ $amcl_status -eq 0 -a \
         $move_base_status -eq 0 -a \
         $path_reco_status -eq 0 -a \
         $drobot_navigation_status -eq 0 -a \
         $map_manage_status -eq 0 ];then
      return 0
    else
      return 1
    fi
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
      roslaunch ${DROBOT_LAUNCH_DIR}/${launch_file} &> ${DROBOT_LOG_DIR}/navigation.log &
      sleep 2
      look
      if [[ $? == 1 ]]; then 
          echo -e "[\e[1;31mFailed: launch ${launch_file}\e[0m]"
          return 1
      else
          echo -e "\e[1;32mExcute Launch [$launch_file] file [Success]\e[0m"
          return 0
      fi
    else 
      echo -e "\e[1;31mFailed: [$launch_file] is Running ... ...\e[0m"
      return 1
    fi
}

function shutdown()
{
    kill_proc ${find_pid_key}
    look
    return $?
}

CALLING_FUNCTION=${1}

${CALLING_FUNCTION}

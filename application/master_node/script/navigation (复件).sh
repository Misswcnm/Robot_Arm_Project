#!/bin/bash

launch_file_name="drobot_navigation.launch"
navigation_launch_pid=0
amcl_pid=0
move_base_pid=0
path_reco_pid=0

function look()
{
    pid=(`pgrep -f ${launch_file_name}`)
    if [ "${pid}" != "" ]; then
        navigation_launch_pid=${pid}
    fi
    pid=(`pgrep -f amcl`)
    if [ "${pid}" != "" ]; then
	     amcl_pid=${pid}
    fi
    pid=(`pgrep -f move_base`)
    if [ "${pid}" != "" ]; then
      move_base_pid=${pid}
    fi
    pid=(`pgrep -f path_reco`)
    if [ "${pid}" != "" ]; then
    	path_reco_pid=${pid}
    fi


    if [ ${navigation_launch_pid} -eq 0 ];then
        echo -e "navigation:  [\e[1;31mSTOPED\e[0m]"
    fi
    if [ ${navigation_launch_pid} -gt 0 ];then
        echo -e "navigation:  [\e[1;32mRUNNING\e[0m]] at pid:[${navigation_launch_pid}]"
    fi

    if [ ${amcl_pid} -eq 0 ]; then
    	echo -e "amcl:  [\e[1;31mSTOPED\e[0m]"
    fi
    if [ ${amcl_pid} -gt 0 ]; then
			echo -e "amcl:  [\e[1;32mRUNNING\e[0m]] at pid:[${amcl_pid}]"
    fi

    if [ ${move_base_pid} -eq 0 ]; then
    	echo -e "move_base: [\e[1;31mSTOPED\e[0m]"
    fi
    if [ ${move_base_pid} -gt 0 ]; then
			echo -e "move_base: [\e[1;32mRUNNING\e[0m]] at pid:[${move_base_pid}]"
    fi

    if [ ${path_reco_pid} -eq 0 ]; then
      echo -e "path_reco: [\e[1;31mSTOPED\e[0m]"
    fi
    if [ ${path_reco_pid} -gt 0 ]; then
      echo -e "path_reco: [\e[1;32mRUNNING\e[0m]] at pid:[${path_reco_pid}]"
    fi

}

function launch()
{
    shutdown
    if [ ${navigation_launch_pid} -eq 0 ]; then
      echo -e "\e[1;34mExcute Launch navigation file... ...\e[0m"
      roslaunch ${DROBOT_LAUNCH_DIR}/${launch_file_name} &> ${DROBOT_LOG_DIR}/navigation.log &
    fi
    sleep 1
    look
}

function shutdown()
{
    echo -e "\e[1;34m**********************************\e[0m"
    look
    if [[ $? == 0 ]];then
      echo -e "\e[1;41m Killing Process...... \e[0m"
      if [ ${navigation_launch_pid} -gt 0 ]; then
          echo "kill -9 ${navigation_launch_pid}"
          kill -9 ${navigation_launch_pid}
      fi
      if [ ${amcl_pid} -gt 0 ]; then
          echo "kill -9 ${amcl_pid}"
          kill -9 ${amcl_pid}
      fi
      if [ ${move_base_pid} -gt 0 ]; then
				  echo "kill -9 ${move_base_pid}"
				  kill -9 ${move_base_pid}
      fi
      if [ ${path_reco_pid} -gt 0 ]; then
          echo "kill -9 ${path_reco_pid}"
          kill -9 ${path_reco_pid}
      fi
      echo -e "\e[1;42m Finished killing Process...... \e[0m"
      look
    fi
}

CALLING_FUNCTION=${1}

${CALLING_FUNCTION}

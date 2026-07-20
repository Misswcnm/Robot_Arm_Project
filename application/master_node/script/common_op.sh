#!/bin/bash
## global value use to return recent get pid
DROBOT_RECENT_PID=0
DROBOT_RECENT_PROC_STOPED=0

function get_proc_pid ()
{
    DROBOT_RECENT_PID=`ps x | grep $1 | grep -v grep | grep -v $0 | awk '{print $1}'`
}
function kill_pids ()
{
    echo "kill $1"
    for pid in $1; do
        echo "kill process:${pid}"
        if [[ ${pid} != "" ]]; then
            kill ${pid}
        fi
    done
}
function kill_proc ()
{
    echo -e "[`date`]\e[1;34m Start Killing [$1] \e[0m"
    get_proc_pid $1
    need_sleep=0
    for pid in ${DROBOT_RECENT_PID}; do
        echo "kill process:${pid}"
        if [ ${pid} != "" ]; then
            kill ${pid}
            need_sleep=1
        fi
    done
    if [ ${need_sleep} == 1 ]; then
        sleep 1
    fi
}
#if is running return 0 or return 1
function look_proc()
{
    get_proc_pid $1
    if [ "${DROBOT_RECENT_PID}" != "" ]; then
        echo -e "[`date`][\e[1;34m$1\e[0m]: [\e[1;32mRUNNING\e[0m] at pid:[${DROBOT_RECENT_PID}]"
        DROBOT_RECENT_PROC_STOPED=0
        return 0
    else
        echo -e "[`date`][\e[1;33m$1\e[0m]: [\e[1;31mSTOPED\e[0m]"
        DROBOT_RECENT_PROC_STOPED=1
        return 1
    fi
}

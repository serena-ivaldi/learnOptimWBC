#!/bin/bash 
ps  | grep -E '(^|\s)[s]h($|\s)' | awk '{print $1 > "pid_clean_wbt.txt"}' 
echo "resetOffset all" | yarp rpc /wholeBodyDynamics/rpc:i
exit 0

#resetOffset all 

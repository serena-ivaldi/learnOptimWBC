#!/bin/bash
# declare STRING variable
 ps  | grep -E '(^|\s)[s]h($|\s)' | awk '{print $1 > "pid.txt"}'

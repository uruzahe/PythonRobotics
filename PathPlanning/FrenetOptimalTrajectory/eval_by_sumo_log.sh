#!/bin/bash
foo=$1;
file=$(echo $foo | cut -d "." -f 1).json
# echo $file
python3 eval_by_sumo_log.py -lfp $1 | tee $file

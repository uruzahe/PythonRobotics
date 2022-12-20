foo=$1
list=(${foo//./ })
echo ${list[0]}.json
python3 eval_by_sumo_log.py -lfp $1 | tee  ${list[0]}.json

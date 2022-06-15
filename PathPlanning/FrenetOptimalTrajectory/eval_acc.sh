#! /bin/zsh

VI="5"
DIM="10"

for VI in "5" "10" "20" "30"
do
  for AI in `seq -4.5 0.5 2.6`
  do
    STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -ai $AI)
    echo "$VI, $AI, $DIM, $STDOUT"
  done
done

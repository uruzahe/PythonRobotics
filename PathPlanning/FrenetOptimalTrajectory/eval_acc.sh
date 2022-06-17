#! /bin/zsh

VI="5"
DIM="10"

for VI in "5" "10" "20" "30"
do
  for AI in `seq -4.5 0.5 2.6`
  do
    for NP in `seq 40 20 100`
    do
      STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -ai $AI --np $NP)
      echo "$VI, $AI, $DIM, $NP, $STDOUT"
    done
  done
done

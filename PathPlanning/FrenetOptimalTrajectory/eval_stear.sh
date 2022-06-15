#! /bin/zsh

VI="5"
DIM="10"

for DIM in "10" "20" "40"
do
  for VI in "5" "10" "20" "30"
  do
    for SI in `seq 0 5 40`
    do
      STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -si $SI)
      echo "$VI, $SI, $DIM, $STDOUT"
    done
  done
done

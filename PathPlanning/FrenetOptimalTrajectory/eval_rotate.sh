for DIM in "10" "20" "30" "40"
do
  for VI in "5" "10" "20" "30"
  do
    for NP in `seq 40 20 100`
    do
      STDOUT=$(python3 kbm.py -vi $VI -dim $DIM --np $NP -rui 1 -rv 90)
      echo "90 deg corner, $VI, $DIM, $NP, $STDOUT"
    done
  done
done

for DIM in "10" "20" "30" "40"
do
  for VI in "5" "10" "20" "30"
  do
    for RV in `seq 10 10 60`
    do
      for RUI in `seq 0.1 0.1 0.5`
      do
        for NP in `seq 40 20 100`
        do
          STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -rui $RUI -rv $RV --np $NP -rt)
          echo "$RV deg S corner, $VI, $DIM, $RUI, $NP, $STDOUT"
        done
      done
    done
  done
done

for DIM in "10" "20" "30" "40"
do
  for VI in "5" "10" "20" "30"
  do
    for RV in `seq 10 10 60`
    do
      for RUI in `seq 0.1 0.1 0.5`
      do
        for NP in `seq 40 20 100`
        do
          STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -rui $RUI -rv $RV --np $NP)
          echo "$RV deg U corner, $VI, $DIM, $RUI, $NP, $STDOUT"
        done
      done
    done
  done
done

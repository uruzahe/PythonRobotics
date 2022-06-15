for DIM in "10" "20"
do
  for VI in "5" "10" "20" "30"
  do
    STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -rui 1 -rv 90)
    echo "90 deg corner, $VI, $DIM, $STDOUT"
  done
done

for DIM in "10" "20"
do
  for VI in "5" "10" "20" "30"
  do
    for RV in `seq 10 10 60`
    do
      for RUI in `seq 0.1 0.1 0.5`
      do
        STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -rui $RUI -rv $RV -rt)
        echo "$RV deg S corner, $VI, $DIM, $RUI, $STDOUT"
      done
    done
  done
done

for DIM in "10" "20"
do
  for VI in "5" "10" "20" "30"
  do
    for RV in `seq 10 10 60`
    do
      for RUI in `seq 0.1 0.1 0.5`
      do
        STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -rui $RUI -rv $RV)
        echo "$RV deg U corner, $VI, $DIM, $RUI, $STDOUT"
      done
    done
  done
done

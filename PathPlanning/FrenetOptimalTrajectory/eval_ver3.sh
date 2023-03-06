RESULT_PATH="./result_prev_for_access.json"
cp /dev/null $RESULT_PATH

for MDIM in "5" "10"
do
  for VI in "10" "20" "30"
  do
    for RV in "0" "45" "90"
    do
      for NP in "50" "100"
      do
        for RUI in `seq 1 2 $((NP/10))`
        do
          for AI in `seq -1.0 1.0 2.0`
          do
            for TI in "0" "45" "90"
            do
              for ETI in "0.1" "0.01"
              do
                python3 eval_prev.py -ai $AI -vi $VI -rui $RUI -rv $RV --np $NP -ti $TI -maxdim $MDIM -rt -eth $ETI --result_path $RESULT_PATH
              done
            done
          done
        done
      done
    done
  done
done

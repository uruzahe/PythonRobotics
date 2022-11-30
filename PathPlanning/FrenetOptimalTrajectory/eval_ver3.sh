for MDIM in "5" "10"
do
  for VI in "15" "30"
  do
    for RV in "0" "45" "90"
    do
      for NP in "50" "100"
      do
        for RUI in `seq 1 1 $((NP/10))`
        do
          for AI in `seq -1.0 1.0 2.0`
          do
            for TI in "0" "45" "90"
            do
              python3 eval_prev.py -ai $AI -vi $VI -rui $RUI -rv $RV --np $NP -ti $TI -maxdim $MDIM -rt -eth 0.1
            done
          done
        done
      done
    done
  done
done

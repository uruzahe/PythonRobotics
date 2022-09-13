echo "acc, speed, rotate_update_interval, point,  duration, angle_init, rotate_angle, maxdim, prop_size, etsi_size, time, file_name"
# echo "$AI, $VI,   $RUI,                   $NP,    $TI,        $RV,          $STDOUT"
# ----- all paterns -----
# for DIM in "5" "10" "15"
# do
#   for VI in "5" "10" "20" "30"
#   do
#     for RV in `seq 0 15 90`
#     do
#       for NP in `seq 40 20 100`
#       do
#         for RUI in `seq 1 1 $((NP/10))`
#         do
#           for AI in `seq -4.5 1.0 2.5`
#           do
#             for TI in `seq 0 30 180`
#             do
#               STDOUT=$(python3 kbm.py -vi $VI -dim $DIM -rui $RUI -rv $RV --np $NP -ti $TI -rt)
#               echo "$AI, $VI, $DIM, $RUI, $NP, $TI, $RV, $STDOUT"
#             done
#           done
#         done
#       done
#     done
#   done
# done

# 2 * 4 * 2 * 10 * 6 * 4
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
              STDOUT=$(python3 kbm.py -ai $AI -vi $VI -rui $RUI -rv $RV --np $NP -ti $TI -maxdim $MDIM -rt )
              # echo "acc, speed, rotate_update_interval, point,  duration,     angle_init, rotate_angle, maxdim, prop_size, etsi_size, time, file_name"
              echo  "$AI, $VI,   $RUI,                   $NP,     $(($NP/10)),         $TI,        $RV,          $MDIM,   $STDOUT"
            done
          done
        done
      done
    done
  done
done

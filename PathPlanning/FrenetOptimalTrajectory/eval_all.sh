echo "acc, speed, dim, rotate_update_interval, point, angle_init, rotate_angle, d_dt, error, file_name"
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


for DIM in "10" "15" "20"
do
  for VI in "15" "30"
  do
    for RV in `seq 0 30 90`
    do
      for NP in "50"
      do
        for RUI in `seq 1 1 $((NP/10))`
        do
          for AI in `seq -1.0 1.0 4.0`
          do
            for TI in "0"
            do
              for D_DT in "0.1"
              do
                STDOUT=$(python3 kbm.py -ai $AI -vi $VI -dim $DIM -rui $RUI -rv $RV --np $NP -ti $TI -rt --d_dt $D_DT)
                echo "$AI, $VI, $DIM, $RUI, $NP, $TI, $RV, $D_DT, $STDOUT"
              done
            done
          done
        done
      done
    done
  done
done

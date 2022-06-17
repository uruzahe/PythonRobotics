#! /bin/zsh
rm ./path_img/*
touch ./path_img/.gitkeep
sh eval_acc.sh > result_acc.csv
# sh eval_stear.sh > result_stear.csv
sh eval_rotate.sh > result_rotate.csv

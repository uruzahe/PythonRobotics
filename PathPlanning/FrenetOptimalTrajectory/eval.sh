#! /bin/zsh
rm ./path_img/*
rm ./result/*
touch ./path_img/.gitkeep
touch ./result/.gitkeep
# sh eval_acc.sh > result_acc.csv
# sh eval_stear.sh > result_stear.csv
# sh eval_rotate.sh > result_rotate.csv
# sh eval_all.sh > result_all.csv
sh eval_ver2.sh | tee result_ver2.csv

#!/bin/bash

files=("data_1_1/config_two.xml" "data_1_1/config_three.xml" "data_2/config_one.xml" "data_2/config_three.xml" "data_3_1/config_one.xml" "data_3_1/config_two.xml")

cd ../build
ls

for f in ${files[@]};do
    echo "Running Experiment For "$f
    now=$(date)
    echo $now
    ./gd_optimizer ../config/$f 
done


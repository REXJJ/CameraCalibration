#!/bin/bash

files=("data_3_1" "data_2" "data_1_1")

cd ../build
ls

for f in ${files[@]};do
    echo "Running Experiment For "$f
    now=$(date)
    echo $now
    touch $f"_output.out"
    ./optimizer ../config/$f/config_initial.xml > $f"_output.out"
done


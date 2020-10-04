#!/bin/bash


echo "Running Sub Class Tests"
cd ../config/new_experiments/sub_class_tests/
for f in *.xml;do
    cd ../../../build/
    echo "Running Experiment For "$f
    now=$(date)
    echo $now
    ./gd_optimizer_ransac ../config/new_experiments/sub_class_tests/$f 
    cd ../config/new_experiments/sub_class_tests/
done

echo "Running Perturbation Tests"
cd ../config/new_experiments/perturbation_tests/
for f in *.xml;do
    cd ../../../build/
    echo "Running Experiment For "$f
    now=$(date)
    echo $now
    ./gd_optimizer_ransac ../config/new_experiments/perturbation_tests/$f 
    cd ../config/new_experiments/perturbation_tests/
done

echo "Running Plane Threshold Tests"
cd ../config/new_experiments/plane_size_tests/
for f in *.xml;do
    cd ../../../build/
    echo "Running Experiment For "$f
    now=$(date)
    echo $now
    ./gd_optimizer_ransac ../config/new_experiments/plane_size_tests/$f 
    cd ../config/new_experiments/plane_size_tests/
done

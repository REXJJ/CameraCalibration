#!/bin/bash
echo "Running Clustering Tests..."
cd ../config/Plane_1/Clusters/
for f in *.xml;do
    cd ../../../build/
    echo "Running Experiment For "$f
    now=$(date)
    echo $now
    ./camera_calibration_optimization ../config/Plane_1/Clusters/$f 
    cd ../config/Plane_1/Clusters/
done


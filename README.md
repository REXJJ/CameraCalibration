# CameraCalibration
Contains methods for calibrating a camera

## Compiling Instruction:

### Packages to install

* Gradient Descent
```
git clone https://github.com/Rookfighter/gradient-descent-cpp.git 
cd gradient-descent-cpp 
mkdir build 
cd build 
cmake .. 
make -j8 
sudo make install
```
* LibNabo
* Qt5
* PCL
* Eigen
* OpenMP

Now this package should be compiled without any hassle.

## Run Instructions:
For visualizing the results, do 
```./pcl_visualizer <config_file>```
See the example to see how the data is organized. The point cloud used for the optimization should be of the form <cloud name>_<number>.<ply/pcd/xyz>, where number identifies the camera location base_T_flange. Common issues include, but not limited to wrong metrics, wrong file locations, etc.
 
The optimizer won't work if the initial estimate is way off. Check this using the visualizer. 

For running the optimizer do the following,
```./camera_calibration_optimization <config file>```

The results can be obtained from the file: results.txt

## Brief Summary of the optimizer

The optimizer needs the following inputs:
Point cloud of a plane taken at various orientations and fk values for the corresponding positions. 
The approximate flange_T_camera transformation.
Few points on the plane taken in the robot base frame - This is done to get an estimate of the true plane. We can also use the input point clouds transformed to the base frame for obtaining this approximation.

The optimizer runs a discrete combinatorial optimization on the flange_T_camera to minimize the error between the true plane and the individual planes.
Then the flange_T_camera transformation is further optimized using gradient descent. 

## License
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)


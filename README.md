# CameraCalibration
Contains methods for calibrating a camera

Compiling Instruction:

- Packages to install

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

- Run Instructions:
For running the optimizer do the following,
./camera_calibration_optimization <config file>

For visualizing the results, do 
./pcl_visualizer <config_file>

Example config files can be found in the config file directory.

- Brief Summary of the optimizer

The optimizer needs the following inputs:
 Point cloud of a plane taken at various orientations and fk values for the corresponding positions. 
 The approximate flange_T_camera transformation.
 Few points on the plane taken in the robot base frame - This is done to get an estimate of the true plane. We can
 also use the input point clouds transformed to the base frame for obtaining this approximation.

 The optimizer runs a discrete combinatorial optimization on the flange_T_camera to minimize the error between the true plane and the individual planes.
 Then the flange_T_camera transformation is further optimized using gradient descent. 


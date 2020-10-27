#include "nabo/nabo.h"
#include "helpers.hpp"
#include <pcl/io/ply_io.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <omp.h>
#include <iostream>
#include <unordered_map>
#include <gdcpp.h>
#include <fstream>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

using namespace Nabo;
using namespace Eigen;
using namespace std;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

std::ofstream outfile;
std::ofstream errorfile;

class Optimizer
{
    private:
        string config_file;
        vector<double> getTransVector(boost::property_tree::ptree &pt,string s);
    public:
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled;
        vector<MatrixXd> inverse_kinematics;
        std::vector<double> flange_transformation_initial;
        std::vector<double> flange_transformation;
        Eigen::Vector4f plane;
        Optimizer(string filename);
        Optimizer(){};
        void getInputs();
        double getError(vector<double>);
        void printError(vector<double>);
        double getError(vector<double>,vector<double>);
        unordered_map<int,int> mapping;
};

Optimizer::Optimizer(string filename)
{
    config_file = filename;
    flange_transformation = std::vector<double>(6,0);
    flange_transformation_initial = std::vector<double>(6,0);
}

vector<double> Optimizer::getTransVector(boost::property_tree::ptree &pt,string s)
{
    string angle_metric = pt.get<std::string>(s+".angle","radian");
    string camera_approx_trans_metric = pt.get<std::string>(s+".metric","m");
    double cam_approx_scale = 1.0;
    if(camera_approx_trans_metric=="cm")
        cam_approx_scale = 100.0;
    else if(camera_approx_trans_metric=="mm")
        cam_approx_scale=1000.0;
    string approx_transformation = pt.get<std::string>(s+".value","0,0,0,0,0,0");
    cout<<approx_transformation<<endl;
    vector<string> coords_str;
    boost::split(coords_str, approx_transformation , boost::is_any_of(","));
    vector<double> coords;
    for(int i=0;i<coords_str.size();i++)
        if(i<3)
            coords.push_back(stof(coords_str[i])/cam_approx_scale);
        else
        {
            double value = stof(coords_str[i]);
            if(angle_metric=="degree")
                value=degreeToRadian(value);
            coords.push_back(value);
        }
    for(auto x:coords)
        cout<<x<<" ";
    cout<<endl;
    return coords;
}

Eigen::Vector4f fitPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  Eigen::MatrixXd lhs (cloud->size(), 3);
  Eigen::VectorXd rhs (cloud->size());
  for (size_t i = 0; i < cloud->size(); ++i)
  {
    const auto& pt = cloud->points[i];
    lhs(i, 0) = pt.x;
    lhs(i, 1) = pt.y;
    lhs(i, 2) = 1.0;

    rhs(i) = -1.0 * pt.z;
  }
  Eigen::Vector3d params = lhs.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(rhs);
  Eigen::Vector3d normal (params(0), params(1), 1.0);
  auto length = normal.norm();
  normal /= length;
  params(2) /= length;
  return {normal(0), normal(1), normal(2), params(2)};
}

double pointToPlaneDistance(Eigen::Vector4f plane, vector<double> pt)
{
    return fabs(plane(0)*pt[0]+plane(1)*pt[1]+plane(2)*pt[2]+plane(3))/(sqrt(pow(plane(0),2)+pow(plane(1),2)+pow(plane(2),2)));
}

double pointToPlaneDistance(vector<double> plane, vector<double> pt)
{
    return fabs(plane[0]*pt[0]+plane[1]*pt[1]+plane[2]*pt[2]+plane[3])/(sqrt(pow(plane[0],2)+pow(plane[1],2)+pow(plane[2],2)));
}

string getSplit(string name, string character,int id)
{
    vector<string> values;
    boost::split(values,name,boost::is_any_of(character));
    if(id<0)
        id = values.size()+id;
    return values[id];
}

int getFileId(string filename)
{
    string file = getSplit(filename,"/",-1);
    file = getSplit(file,".",0);
    int number = stoi(getSplit(file,"_",1));
    return number;
}

void Optimizer::getInputs()
{
    ifstream file;
    std::cout<<"Config File: "<<config_file<<endl;
    file.open(config_file);
    using boost::property_tree::ptree;
    ptree pt;
    read_xml(file, pt);
    string camera_metric = pt.get<std::string>("data.camera.metric","m");
    int counter = 0;
    for (const auto &cloud : pt.get_child("data.camera.clouds"))
    {
        string filename = cloud.second.data();
        int cloud_id = getFileId(filename); 
        cout<<"Cloud Number: "<<cloud_id<<endl;
        mapping[counter++] = cloud_id-1;
        ifstream file(filename);
        string line;
        for(int c=0;c<14&&getline(file,line);c++);
        PointCloudT::Ptr pointcloud(new PointCloudT);
        while(getline(file,line))
        {
            vector<string> values_from_file;
            boost::split(values_from_file, line, boost::is_any_of(" "));
            pcl::PointXYZRGB pt;
            pt.x = stof(values_from_file[0]);
            pt.y = stof(values_from_file[1]);
            pt.z = stof(values_from_file[2]);
            pt.r = stof(values_from_file[3]);
            pt.g = stof(values_from_file[4]);
            pt.b = stof(values_from_file[5]);
            pointcloud->points.push_back(pt);
        }
        PointCloudT::Ptr temp_cloud(new PointCloudT);
        for(int i=0;i<pointcloud->points.size();i++)
        {
            auto pt = pointcloud->points[i];
            if(pt.x==0.0&&pt.y==0.0&&pt.z==0.0)
                continue;
            temp_cloud->points.push_back(pt);
        }
        clouds.push_back(temp_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw_temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*temp_cloud,*cloud_bw_temp);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_bw_temp);
        constexpr double leaf = 0.02f;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.filter (*cloud_filtered);
        cout<<"Filtered Clouds Size: "<<cloud_filtered->points.size()<<endl;
        cloud_downsampled.push_back(cloud_filtered);
    }
    string ik_filename  = pt.get<std::string>("data.camera.transformations.inverse_kinematics.location");
    cout<<ik_filename<<endl;
    string transformation_metric = pt.get<std::string>("data.camera.transformations.inverse_kinematics.metric","m");
    inverse_kinematics = readTransformations(ik_filename,true,transformation_metric);
    string touch_points_file = pt.get<std::string>("data.plane.file","");
    pcl::PointCloud<pcl::PointXYZ>::Ptr touch_points(new pcl::PointCloud<pcl::PointXYZ>);
    ifstream file_h(touch_points_file);
    string line;
    while(getline(file_h,line)&&line.size())
    {
        vector<string> v;
        split(v,line,boost::is_any_of(","));
        pcl::PointXYZ pt;
        pt.x = stof(v[0])/1000.0;
        pt.y = stof(v[1])/1000.0;
        pt.z = stof(v[2])/1000.0;
        touch_points->points.push_back(pt);
    }
    cout<<"Size of touch points: "<<touch_points->points.size()<<endl;
    plane = fitPlane(touch_points);
    cout<<"Plane Equation: "<<endl;
    for(int i=0;i<4;i++)
        cout<<plane(i)<<" ";
    cout<<endl;
    cout<<"Transformations Read"<<endl;
    auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_flange.size();i++)
    {
        flange_transformation[i]=transformation_initial_flange[i];
        flange_transformation_initial[i]=transformation_initial_flange[i];
    }
}

double Optimizer::getError(vector<double> transformation)
{
    double error = 0.0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    // vector<double> error_vec(clouds.size(),0);
    for(int j=0;j<clouds.size();j++)
    {
        double average = 0.0;
        double error_mx = -1e9;
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[mapping[j]]*cam_T_flange;
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        for(int i=0;i<cloud_downsampled[j]->points.size();i++)
        {
            auto pt = cloud_downsampled[j]->points[i];
            float src[3];
            float out[3];
            src[0] = pt.x;
            src[1] = pt.y;
            src[2] = pt.z;
            apply_transformation_optimized(src,out,trans);
            double distance = pointToPlaneDistance(plane,{out[0],out[1],out[2]});
            average+=distance;
            if(distance>error_mx)
                error_mx = distance;
        }
        error+=average/cloud_downsampled[j]->points.size();
        // error+=error_mx;
    }
    return error/clouds.size();
}
double Optimizer::getError(vector<double> transformation,vector<double> plane_local)
{
    double error = 0.0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    // vector<double> error_vec(clouds.size(),0);
    for(int j=0;j<clouds.size();j++)
    {
        double average = 0.0;
        double error_mx = -1e9;
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[mapping[j]]*cam_T_flange;
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        for(int i=0;i<cloud_downsampled[j]->points.size();i++)
        {
            auto pt = cloud_downsampled[j]->points[i];
            float src[3];
            float out[3];
            src[0] = pt.x;
            src[1] = pt.y;
            src[2] = pt.z;
            apply_transformation_optimized(src,out,trans);
            double distance = pointToPlaneDistance(plane_local,{out[0],out[1],out[2]});
            average+=distance;
            if(distance>error_mx)
                error_mx = distance;
        }
        error+=average/cloud_downsampled[j]->points.size();
        // error+=error_mx;
    }
    return error/clouds.size();
}
void Optimizer::printError(vector<double> transformation)
{
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    for(int j=0;j<clouds.size();j++)
    {
        double average = 0.0;
        double error_mx = -1e9;
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[mapping[j]]*cam_T_flange;
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        for(int i=0;i<cloud_downsampled[j]->points.size();i++)
        {
            auto pt = cloud_downsampled[j]->points[i];
            float src[3];
            float out[3];
            src[0] = pt.x;
            src[1] = pt.y;
            src[2] = pt.z;
            apply_transformation_optimized(src,out,trans);
            double distance = pointToPlaneDistance(plane,{out[0],out[1],out[2]});
            average+=distance;
            if(distance>error_mx)
                error_mx = distance;
        }
        // error+=average/cloud_downsampled[j]->points.size();
        errorfile<<"Downsampled No: "<<j<<" Error Avg: "<<average/cloud_downsampled[j]->points.size()*1000.0<<" Errom Max: "<<error_mx*1000.0<<endl;
    }
    
    for(int j=0;j<clouds.size();j++)
    {
        double average = 0.0;
        double error_mx = -1e9;
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[mapping[j]]*cam_T_flange;
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        for(int i=0;i<clouds[j]->points.size();i++)
        {
            auto pt = clouds[j]->points[i];
            float src[3];
            float out[3];
            src[0] = pt.x;
            src[1] = pt.y;
            src[2] = pt.z;
            apply_transformation_optimized(src,out,trans);
            double distance = pointToPlaneDistance(plane,{out[0],out[1],out[2]});
            average+=distance;
            if(distance>error_mx)
                error_mx = distance;
        }
        // error+=average/cloud_downsampled[j]->points.size();
        errorfile<<"Real No: "<<j<<" Error Avg: "<<average/clouds[j]->points.size()*1000.0<<" Errom Max: "<<error_mx*1000.0<<endl;
    }
    errorfile<<"--------------------------------------------"<<endl;
}

Optimizer opti;

void gradientDescent()
{
    struct Functor
    {
        Functor()
        { 
        }

        double operator()(const Eigen::VectorXd &xval, Eigen::VectorXd &) const
        {
            vector<double> trans(6);
            for(int i=0;i<6;i++){
                trans[i]=xval(i);
            }
            return opti.getError(trans);
        }
    };
    gdc::GradientDescent<double, Functor,
        gdc::WolfeBacktracking<double>> optimizer;

    optimizer.setMaxIterations(10000);
    optimizer.setMinGradientLength(1e-6);
    optimizer.setMinStepLength(1e-6);
    optimizer.setMomentum(0.4);
    optimizer.setVerbosity(4);
    Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(6);
    for(int i=0;i<6;i++)
    {
        initialGuess(i) = opti.flange_transformation[i];
    }
    cout<<"Plane Value"<<endl;
    for(int i=0;i<4;i++)
        cout<<opti.plane(i)<<" ";
    cout<<endl;
    auto result = optimizer.minimize(initialGuess);
    std::cout << "Done! Converged: " << (result.converged ? "true" : "false")
        << " Iterations: " << result.iterations << std::endl;
    std::cout << "Final fval: " << result.fval << std::endl;
    std::cout << "Final xval: " << result.xval.transpose() << std::endl;
    std::cout<<"---------------------------------------------------------"<<endl;
    outfile<<"Gradient Descent on Flange Transformation..."<<endl;
    outfile<<"Iterations: "<<result.iterations<<" Converged: "<<(result.converged ? "true" : "false")<<" Final fval: "<<result.fval<<endl;
    outfile<<"Flange Transformation"<<endl;
    auto result_flange_trans = result.xval.transpose().head(6);
    for(int i=0;i<5;i++)
        outfile<<result_flange_trans(i)<<", ";
    outfile<<result_flange_trans(5)<<endl;
    outfile<<"Plane Equation"<<endl;
    for(int i=0;i<3;i++)
        outfile<<opti.plane(i)<<", ";
    outfile<<opti.plane(3)<<endl;
    for(int i=0;i<6;i++)
        opti.flange_transformation[i] = result_flange_trans(i);
    opti.printError({result_flange_trans(0),result_flange_trans(1),result_flange_trans(2),result_flange_trans(3),result_flange_trans(4),result_flange_trans(5)});
}
void gradientDescentWithPlane()
{
    struct Functor
    {
        Functor()
        { 
        }

        double operator()(const Eigen::VectorXd &xval, Eigen::VectorXd &) const
        {
            vector<double> trans(6);
            vector<double> plane(4);
            for(int i=0;i<6;i++){
                trans[i]=xval(i);
            }
            for(int i=6;i<10;i++)
                plane[i-6]=xval(i);
            return opti.getError(trans,plane);
        }
    };
    gdc::GradientDescent<double, Functor,
        gdc::WolfeBacktracking<double>> optimizer;

    optimizer.setMaxIterations(10000);
    optimizer.setMinGradientLength(1e-6);
    optimizer.setMinStepLength(1e-6);
    optimizer.setMomentum(0.4);
    optimizer.setVerbosity(4);
    Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(10);
    for(int i=0;i<6;i++)
    {
        initialGuess(i) = opti.flange_transformation[i];
    }
    cout<<"Plane Value"<<endl;
    for(int i=0;i<4;i++)
        cout<<opti.plane(i)<<" ";
    cout<<endl;
    for(int i=0;i<4;i++)
        initialGuess(i+6) = opti.plane(i);
    auto result = optimizer.minimize(initialGuess);
    std::cout << "Done! Converged: " << (result.converged ? "true" : "false")
        << " Iterations: " << result.iterations << std::endl;
    std::cout << "Final fval: " << result.fval << std::endl;
    std::cout << "Final xval: " << result.xval.transpose() << std::endl;
    std::cout<<"---------------------------------------------------------"<<endl;
    outfile<<"Gradient Descent on Plane..."<<endl;
    outfile<<"Iterations: "<<result.iterations<<" Converged: "<<(result.converged ? "true" : "false")<<" Final fval: "<<result.fval<<endl;
    outfile<<"Flange Transformation"<<endl;
    auto result_flange_trans = result.xval.transpose().head(6);
    for(int i=0;i<5;i++)
        outfile<<result_flange_trans(i)<<", ";
    outfile<<result_flange_trans(5)<<endl;
    outfile<<"Plane Equation"<<endl;
    auto result_plane = result.xval.transpose().tail(4);
    for(int i=0;i<3;i++)
        outfile<<result_plane(i)<<", ";
    outfile<<result_plane(3)<<endl;
    opti.printError({result_flange_trans(0),result_flange_trans(1),result_flange_trans(2),result_flange_trans(3),result_flange_trans(4),result_flange_trans(5)});
}

void discreteCombinatorialOptimization()
{
    vector<double> transformation = opti.flange_transformation;
    cout<<"True: "<<opti.getError(transformation);
    vector<double> transformation_best = opti.flange_transformation;
    double error_min = 1e9;
    double t_min = -20, t_max = 20, r_min = -5, r_max = 5;
    for(double x = t_min;x<=t_max;x+=4)
        for(double y = t_min;y<=t_max;y+=4)
            for(double z = t_min;z<=t_max;z+=4)
                for(double zt=r_min;zt<=r_max;zt++)
                    for(double yt=r_min;yt<=r_max;yt++)
                        for(double xt=r_min;xt<=r_max;xt++)
                        {
                            vector<double> temp_transformation = {transformation[0]+x/1000.0,transformation[1]+y/1000.0,transformation[2]+z/1000.0,transformation[3]+degreeToRadian(zt),transformation[4]+degreeToRadian(yt),transformation[5]+degreeToRadian(xt)};
                            double total_error = opti.getError(temp_transformation);
                            if(total_error<error_min)
                            {
                                cout<<"Error: "<<total_error<<endl;
                                transformation_best = temp_transformation;
                                error_min = total_error;
                            }
                        }
    cout<<"Best Transformation:"<<endl;
    for(auto x:transformation_best)
        cout<<x<<" ";
    cout<<endl;
    outfile<<"Flange Transformation From Discrete Optimization"<<endl;
    for(int i=0;i<5;i++)
        outfile<<transformation_best[i]<<", ";
    outfile<<transformation_best[5]<<endl;
    opti.flange_transformation = transformation_best;
}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    outfile.open("results.txt", std::ios_base::app); // append instead of overwrite
    string config_filename = argv[1];
    errorfile.open("new_experiments_errors.txt", std::ios_base::app); // append instead of overwrite
    config_filename = argv[1];
    outfile<<"Results: "<<config_filename<<endl;
    errorfile<<"Results: "<<config_filename<<endl;
    opti = Optimizer(config_filename);
    opti.getInputs();
    cout<<"Starting optimization"<<endl;
    discreteCombinatorialOptimization();
    gradientDescent();
    gradientDescentWithPlane();
    outfile<<"------------------------------------------------------"<<endl;
    return 0;
}


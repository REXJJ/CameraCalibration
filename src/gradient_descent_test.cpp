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
#include <gdcpp.h>

using namespace Nabo;
using namespace Eigen;
using namespace std;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Optimizer
{
    private:
        string config_file;
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled;
        vector<MatrixXd> inverse_kinematics;
        std::vector<double> transformation_initial;
        std::vector<double> flange_transformation_initial;
        vector<double> getTransVector(boost::property_tree::ptree &pt,string s);

        int K ;
        MatrixXf M;
        NNSearchF* nns;
        vector<MatrixXf> Ns;
        vector<MatrixXi> indis;
        vector<MatrixXf> dists;

    public:
        std::vector<double> transformation;
        std::vector<double> flange_transformation;
        Optimizer(string filename);
        Optimizer(){};
        void getInputs();
        double getError(vector<double> ,vector<double> );
        double getErrorObj(vector<double> );
};

Optimizer::Optimizer(string filename)
{
    config_file = filename;
    cloud.reset (new PointCloudT);
    transformation = std::vector<double>(6,0);
    flange_transformation = std::vector<double>(6,0);
    transformation_initial = std::vector<double>(6,0);
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
void Optimizer::getInputs()
{
    ifstream file;
    std::cout<<"Config File: "<<config_file<<endl;
    file.open(config_file);
    using boost::property_tree::ptree;
    ptree pt;
    read_xml(file, pt);
    string camera_metric = pt.get<std::string>("data.camera.metric","m");
    cout<<"------------- "<<camera_metric<<endl;
    for (const auto &cloud : pt.get_child("data.camera.clouds"))
    {
        string filename = cloud.second.data();
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
        constexpr double leaf = 0.0225f;
        // constexpr double leaf = 0.007f;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.filter (*cloud_filtered);
        cloud_downsampled.push_back(cloud_filtered);
        // PointCloudT::Ptr pointcloud_output(new PointCloudT);
        // cloud_outputs.push_back(pointcloud_output);
        cout<<filename<<endl;
    }
    string ik_filename  = pt.get<std::string>("data.camera.transformations.inverse_kinematics");
    cout<<ik_filename<<endl;
    inverse_kinematics = readTransformations(ik_filename,true);
    cout<<"Transformations Read"<<endl;
    //Reading the scan cloud.
    string object_metric = pt.get<std::string>("data.scan.metric","m");
    cout<<"------------- "<<object_metric <<endl;
    for(const auto &cloud_location :pt.get_child("data.scan.clouds"))
    {
        string filename = cloud_location.second.data();
        readPointCloud(filename,cloud,object_metric );
    }
    cout<<"Scan read"<<endl; 
    //Reading Default Values;
    auto transformation_initial_object= getTransVector(pt,"data.scan.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_object.size();i++)
    {
        transformation[i]=transformation_initial_object[i];
        transformation_initial[i] = transformation_initial_object[i];
    }
    auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_flange.size();i++)
    {
        flange_transformation[i]=transformation_initial_flange[i];
        flange_transformation_initial[i]=transformation_initial_flange[i];
    }
    cout<<"Here"<<endl;


    K = 1;
    M = MatrixXf::Zero(3, cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    nns = NNSearchF::createKDTreeLinearHeap(M);
    for(int i=0;i<clouds.size();i++)
    {
        MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[i]->points.size());
        Ns.push_back(N);
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, Ns[i].cols());
        dists2.resize(1, Ns[i].cols());
        indis.push_back(indices);
        dists.push_back(dists2);
        cout<<"Points Size: "<<cloud_downsampled[i]->points.size()<<endl;
    }
}

double Optimizer::getError(vector<double> flange_transformation,vector<double> transformation)
{
    double error = 0.0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    for(int j=0;j<clouds.size();j++)
    {
        float average = 0.0;
        float maximum = -1e9;
        int counter = 0;
        Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
        world_T_object = world_T_object.inverse();
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        Eigen::Affine3d transW;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                transW(a,b) = world_T_object(a,b);
        for(int i=0;i<cloud_downsampled[j]->points.size();i++)
        {
            auto point = cloud_downsampled[j]->points[i];
#if 1
            float src[3];
            float out[3];
            src[0] = point.x;
            src[1] = point.y;
            src[2] = point.z;
            apply_transformation_optimized(src,out,trans);
            src[0] = out[0];
            src[1] = out[1];
            src[2] = out[2];
            apply_transformation_optimized(src,out,transW);
            Ns[j](0,i) = out[0];
            Ns[j](1,i) = out[1];
            Ns[j](2,i) = out[2];
#else
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            pts=apply_transformation(pts,transformation);
            pts=apply_transformation(pts,world_T_object);
            Ns[j](0,i) = pts(0,0);
            Ns[j](1,i) = pts(0,1);
            Ns[j](2,i) = pts(0,2);
#endif
            counter++;
        }
        nns->knn(Ns[j], indis[j],dists[j], 1, 0.1, NNSearchF::SORT_RESULTS);
        for(int i=0;i<counter;i++)
        {
            double distance = sqrt(dists[j](0,i));
            if(maximum<distance)
                maximum=distance;
            average+=distance;
        }
        average=average/counter;
        error = error + average;
    }
    return error/clouds.size();
}

double Optimizer::getErrorObj(vector<double> trans_obj)
{
    double error = 0.0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    for(int j=0;j<clouds.size();j++)
    {
        float average = 0.0;
        float maximum = -1e9;
        int counter = 0;
        Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(trans_obj);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
        world_T_object = world_T_object.inverse();
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        Eigen::Affine3d transW;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                transW(a,b) = world_T_object(a,b);
        for(int i=0;i<cloud_downsampled[j]->points.size();i++)
        {
            auto point = cloud_downsampled[j]->points[i];
#if 1
            float src[3];
            float out[3];
            src[0] = point.x;
            src[1] = point.y;
            src[2] = point.z;
            apply_transformation_optimized(src,out,trans);
            src[0] = out[0];
            src[1] = out[1];
            src[2] = out[2];
            apply_transformation_optimized(src,out,transW);
            Ns[j](0,i) = out[0];
            Ns[j](1,i) = out[1];
            Ns[j](2,i) = out[2];
#else
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            pts=apply_transformation(pts,transformation);
            pts=apply_transformation(pts,world_T_object);
            Ns[j](0,i) = pts(0,0);
            Ns[j](1,i) = pts(0,1);
            Ns[j](2,i) = pts(0,2);
#endif
            counter++;
        }
        nns->knn(Ns[j], indis[j],dists[j], 1, 0.1, NNSearchF::SORT_RESULTS);
        for(int i=0;i<counter;i++)
        {
            double distance = sqrt(dists[j](0,i));
            if(maximum<distance)
                maximum=distance;
            average+=distance;
        }
        average=average/counter;
        error = error + average;
    }
    return error/clouds.size();
}

string k_Filename;
Optimizer opti;

void gradientDescentObject()
{
    struct Ackley
    {
        Ackley()
        { 
        }

        double operator()(const Eigen::VectorXd &xval, Eigen::VectorXd &) const
        {
            vector<double> flange_trans(6),trans(6);
            for(int i=0;i<6;i++){
                trans[i]=xval(i);
            }
            return opti.getErrorObj(trans);
        }
    };
    gdc::GradientDescent<double, Ackley,
        gdc::WolfeBacktracking<double>> optimizer;

    optimizer.setMaxIterations(200);
    optimizer.setMinGradientLength(1e-6);
    optimizer.setMinStepLength(1e-6);
    optimizer.setMomentum(0.4);
    optimizer.setVerbosity(4);
    Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(6);
    for(int i=0;i<6;i++)
    {
        // initialGuess(i) = opti.flange_transformation[i];
        initialGuess(i) = opti.transformation[i];
    }
    auto result = optimizer.minimize(initialGuess);
    std::cout << "Done! Converged: " << (result.converged ? "true" : "false")
        << " Iterations: " << result.iterations << std::endl;
    std::cout << "Final fval: " << result.fval << std::endl;
    std::cout << "Final xval: " << result.xval.transpose() << std::endl;
}

void gradientDescent()
{
    struct Ackley
    {
        Ackley()
        { 
        }

        double operator()(const Eigen::VectorXd &xval, Eigen::VectorXd &) const
        {
            vector<double> flange_trans(6),trans(6);
            for(int i=0;i<6;i++){
                flange_trans[i]=xval(i);
                trans[i]=xval(6+i);
            }
            return opti.getError(flange_trans,trans);
        }
    };
    gdc::GradientDescent<double, Ackley,
        gdc::WolfeBacktracking<double>> optimizer;

    optimizer.setMaxIterations(200);
    optimizer.setMinGradientLength(1e-6);
    optimizer.setMinStepLength(1e-6);
    optimizer.setMomentum(0.4);
    optimizer.setVerbosity(4);
    Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(12);
    for(int i=0;i<6;i++)
    {
        initialGuess(i) = opti.flange_transformation[i];
        initialGuess(6+i) = opti.transformation[i];
    }
    auto result = optimizer.minimize(initialGuess);
    std::cout << "Done! Converged: " << (result.converged ? "true" : "false")
        << " Iterations: " << result.iterations << std::endl;
    std::cout << "Final fval: " << result.fval << std::endl;
    std::cout << "Final xval: " << result.xval.transpose() << std::endl;
}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    k_Filename = argv[1];
    opti = Optimizer(k_Filename);
    opti.getInputs();
    gradientDescentObject();
    // gradientDescent();
    return 0;
}


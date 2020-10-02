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
        double getError(vector<double>);
        vector<double> plane_;
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;
};

Optimizer::Optimizer(string filename)
{
    config_file = filename;
    cloud.reset (new PointCloudT);
    transformation = std::vector<double>(6,0);
    plane_ = std::vector<double>(4,0);
    flange_transformation = std::vector<double>(6,0);
    transformation_initial = std::vector<double>(6,0);
    flange_transformation_initial = std::vector<double>(6,0);
    combined_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
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

Eigen::MatrixXd getPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    Eigen::MatrixXd parameters(1,4);
    parameters<<coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3];
    return parameters;
}

double pointToPlaneDistance(vector<double> plane, vector<double> pt)
{
    return fabs(plane[0]*pt[0]+plane[1]*pt[1]+plane[2]*pt[2]+plane[3])/(sqrt(pow(plane[0],2)+pow(plane[1],2)+pow(plane[2],2)));
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
#if 0
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_bw_temp);
        constexpr double leaf = 0.0225f;
        // constexpr double leaf = 0.007f;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.filter (*cloud_filtered);
#else
        auto plane = getPlane(cloud_bw_temp);
        for(auto pt:cloud_bw_temp->points)
        {
            if(pointToPlaneDistance({plane(0),plane(1),plane(2),plane(3)},{pt.x,pt.y,pt.z})<0.000001)
            {
                cloud_filtered->points.push_back(pt);
            }
        }
#endif
        cout<<"Filtered Clouds Size: "<<cloud_filtered->points.size()<<endl;
        cloud_downsampled.push_back(cloud_filtered);
        // PointCloudT::Ptr pointcloud_output(new PointCloudT);
        // cloud_outputs.push_back(pointcloud_output);
        cout<<filename<<endl;
    }
    string ik_filename  = pt.get<std::string>("data.camera.transformations.inverse_kinematics");
    cout<<ik_filename<<endl;
    inverse_kinematics = readTransformations(ik_filename,true);
    cout<<"Transformations Read"<<endl;
    auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_flange.size();i++)
    {
        flange_transformation[i]=transformation_initial_flange[i];
        flange_transformation_initial[i]=transformation_initial_flange[i];
    }
    
    for(int j=0;j<clouds.size();j++)
    {
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        for(int i=0;i<clouds[j]->points.size();i++)
        {
            auto point = clouds[j]->points[i];
#if 1
            float src[3];
            float out[3];
            src[0] = point.x;
            src[1] = point.y;
            src[2] = point.z;
            apply_transformation_optimized(src,out,trans);
            pcl::PointXYZ pt;
            pt.x  = out[0];
            pt.y =  out[1];
            pt.z =  out[2];
            combined_cloud->points.push_back(pt);
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
        }
    }
}



double Optimizer::getError(vector<double> transformation,vector<double> plane)
{
    double error = 0.0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    for(int j=0;j<clouds.size();j++)
    {
        double average = 0.0;
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
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
            average+=pointToPlaneDistance(plane,{out[0],out[1],out[2]});
        }
        error+=average/cloud_downsampled[j]->points.size();
    }
    return error/clouds.size();
}

double Optimizer::getError(vector<double> transformation)
{
    double error = 0.0;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    for(int j=0;j<clouds.size();j++)
    {
        double average = 0.0;
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(transformation);
        Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
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
            average+=pointToPlaneDistance(plane_,{out[0],out[1],out[2]});
        }
        error+=average/cloud_downsampled[j]->points.size();
    }
    return error/clouds.size();
}



string k_Filename;
Optimizer opti;

void gradientDescent()
{
    struct Ackley
    {
        Ackley()
        { 
        }

        double operator()(const Eigen::VectorXd &xval, Eigen::VectorXd &) const
        {
            vector<double> trans(6);
            vector<double> plane(4);
            for(int i=0;i<6;i++){
                trans[i]=xval(i);
            }
            // for(int i=6;i<10;i++)
            //     plane[i-6]=xval(i);
            return opti.getError(trans);
        }
    };
    // gdc::GradientDescent<double, Ackley,
    //     gdc::WolfeBacktracking<double>> optimizer;

    gdc::GradientDescent<double, Ackley,
        gdc::ConstantStepSize<double>> optimizer;

    optimizer.setStepSize(gdc::ConstantStepSize<double>(0.000001));
    optimizer.setMaxIterations(200);
    optimizer.setMinGradientLength(1e-9);
    optimizer.setMinStepLength(1e-9);
    optimizer.setMomentum(0.4);
    optimizer.setVerbosity(4);
    Eigen::VectorXd initialGuess = Eigen::VectorXd::Zero(6);
    for(int i=0;i<6;i++)
    {
        initialGuess(i) = opti.flange_transformation[i];
    }
    auto plane = getPlane(opti.combined_cloud);
    assert(plane(0)==plane(0,0));
    assert(plane(1)==plane(0,1));
    assert(plane(2)==plane(0,2));
    assert(plane(3)==plane(0,3));
    cout<<"Plane Value"<<endl;
    for(int i=0;i<4;i++)
        cout<<plane(i)<<" ";
    for(int i=0;i<4;i++)
        opti.plane_[i]=plane(i);
    cout<<endl;
    // for(int i=0;i<4;i++)
    //     initialGuess(i+6) = plane(i);
    //Get the initial estimate of the plane here and feed it to the optimizer.

    auto result = optimizer.minimize(initialGuess);
    std::cout << "Done! Converged: " << (result.converged ? "true" : "false")
        << " Iterations: " << result.iterations << std::endl;
    std::cout << "Final fval: " << result.fval << std::endl;
    std::cout << "Final xval: " << result.xval.transpose() << std::endl;
    std::cout<<"---------------------------------------------------------"<<endl;
    
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0.2,0.2,0.2);
    viewer->initCameraParameters ();
    viewer->addPointCloud<pcl::PointXYZ>(opti.combined_cloud);
    pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);

    auto new_plane = result.xval.transpose();

    plane_1->values.resize (4);
    plane_1->values[0] =opti.plane_[0];
    plane_1->values[1] =opti.plane_[1];
    plane_1->values[2] =opti.plane_[2];
    plane_1->values[3] =opti.plane_[3];
    viewer->addPlane (*plane_1, "plane_1", 0);
    while(!viewer->wasStopped())
            viewer->spinOnce(100);

}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    outfile.open("gd_outputs.txt", std::ios_base::app); // append instead of overwrite
    outfile << "Results: \n"; 
    k_Filename = argv[1];
    outfile<<k_Filename<<endl;
    opti = Optimizer(k_Filename);
    opti.getInputs();
    gradientDescent();
    return 0;
}


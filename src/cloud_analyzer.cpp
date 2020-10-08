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
std::ofstream errorfile;

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

    public:
        std::vector<double> transformation;
        std::vector<double> flange_transformation;
        Optimizer(string filename);
        Optimizer(){};
        void getInputs();
        pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud;
};

Optimizer::Optimizer(string filename)
{
    config_file = filename;
    cloud.reset (new PointCloudT);
    transformation = std::vector<double>(6,0);
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
        double distance_threshold = pt.get<double>("data.camera.threshold",0.000001);
        cout<<distance_threshold<<"-------------------------"<<endl;
        auto plane = fitPlane(cloud_bw_temp);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
        static int counter = 0;
        counter++;
        for(auto pt:cloud_bw_temp->points)
        {
            auto distance_to_plane = pointToPlaneDistance({plane(0),plane(1),plane(2),plane(3)},{pt.x,pt.y,pt.z});
            pcl::PointXYZRGB pt_temp;
            pt_temp.x = pt.x;
            pt_temp.y = pt.y;
            pt_temp.z = pt.z;
            if(distance_to_plane*1000.0<1.0)
                pt_temp.g=255;
            else if(distance_to_plane*1000.0<2.5)
                pt_temp.b=255;
            else if(distance_to_plane*1000.0<3.5)
            {
                pt_temp.b=100;
                pt_temp.r=100;
                pt_temp.g=100;
            }
            else 
                pt_temp.r=255;
            cloud_temp->points.push_back(pt_temp);
            if(distance_to_plane<distance_threshold)
            {
                cloud_filtered->points.push_back(pt);
            }
        }
        if(counter==1||counter==2||counter==4||counter==33)
        {
            pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
            viewer->setBackgroundColor (1.0,1.0,1.0);
            viewer->initCameraParameters ();
            viewer->addPointCloud<pcl::PointXYZRGB>(cloud_temp);
            pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);
            plane_1->values.resize (4);
            plane_1->values[0] = plane(0);
            plane_1->values[1] = plane(1);
            plane_1->values[2] = plane(2);
            plane_1->values[3] = plane(3);
            // viewer->addPlane (*plane_1, "plane_1", 0);
            while(!viewer->wasStopped())
                viewer->spinOnce(100);
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

string k_Filename;
Optimizer opti;

int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    outfile.open("new_experiments.txt", std::ios_base::app); // append instead of overwrite
    k_Filename = argv[1];
    errorfile.open("new_experiments_errors.txt", std::ios_base::app); // append instead of overwrite
    k_Filename = argv[1];
    outfile<<"Results: "<<k_Filename<<endl;
    errorfile<<"Results: "<<k_Filename<<endl;
    opti = Optimizer(k_Filename);
    opti.getInputs();
    return 0;
}


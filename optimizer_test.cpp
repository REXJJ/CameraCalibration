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
        std::vector<double> transformation;
        std::vector<double> flange_transformation;
        std::vector<double> transformation_initial;
        std::vector<double> flange_transformation_initial;
        vector<double> getTransVector(boost::property_tree::ptree &pt,string s);
    public:
        Optimizer(string filename);
        void getInputs();
        // void findSeedPoints();
        // void findSeedPointsParallel();
        // void findSeedPointsPar();
        void discreteCombintorialOptimizer();
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
        PointCloudT::Ptr pointcloud(new PointCloudT);
        readPointCloud(filename,pointcloud,camera_metric);
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
        constexpr double leaf = 0.007f;
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
}

// void Optimizer::findSeedPoints()
// {
//     int K = 1;
//     MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//     for(int i=0;i<cloud->points.size();i++)
//     {
//         auto pt = cloud->points[i];
//         M(0,i) = pt.x;
//         M(1,i) = pt.y;
//         M(2,i) = pt.z;
//     }
//     NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//
//     vector<double> error_avg(clouds.size(),1.0/0.0);
//     vector<double> error_max(clouds.size(),1.0/0.0);
//     double max_error = -1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     // #pragma omp parallel
//     // #pragma omp for
//     TIC();
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             N(0,i) = out[0];
//             N(1,i) = out[1];
//             N(2,i) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//             counter++;
//         }
//         MatrixXi indices;
//         MatrixXf dists2;
//         indices.resize(1, N.cols());
//         dists2.resize(1, N.cols());
//         nns->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);
//
//         for(int i=0;i<counter;i++)
//         {
//             double distance = sqrt(dists2(0,i));
//             if(maximum<distance)
//                 maximum=distance;
//             average+=distance;
//         }
//         average=average/counter;
//         error_avg[j]=average;
//         error_max[j]=maximum;
//     }
//     TOC();
//     cout<<"Errors: "<<endl;
//     for(int i=0;i<error_avg.size();i++)
//     {
//         cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
//     }
// }
// void Optimizer::findSeedPointsPar()
// {
//     int K = 1;
//     vector<NNSearchF*> nabos;
//     vector<MatrixXf> mats;
//     for(int i=0;i<clouds.size();i++)
//     {
//         MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//         for(int i=0;i<cloud->points.size();i++)
//         {
//             auto pt = cloud->points[i];
//             M(0,i) = pt.x;
//             M(1,i) = pt.y;
//             M(2,i) = pt.z;
//         }
//         NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//         mats.push_back(M);
//         nabos.push_back(nns);
//     }
//     vector<double> error_avg(clouds.size(),1.0/0.0);
//     vector<double> error_max(clouds.size(),1.0/0.0);
//     double max_error = -1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     TIC();
//     #pragma omp parallel
//     #pragma omp for
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             N(0,i) = out[0];
//             N(1,i) = out[1];
//             N(2,i) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//             counter++;
//         }
//         MatrixXi indices;
//         MatrixXf dists2;
//         indices.resize(1, N.cols());
//         dists2.resize(1, N.cols());
//         nabos[j]->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);
//         for(int i=0;i<counter;i++)
//         {
//             double distance = sqrt(dists2(0,i));
//             if(maximum<distance)
//                 maximum=distance;
//             average+=distance;
//         }
//         average=average/counter;
//         error_avg[j]=average;
//         error_max[j]=maximum;
//     }
//     TOC();
//     cout<<"Errors: "<<endl;
//     for(int i=0;i<error_avg.size();i++)
//     {
//         cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
//     }
// }
// void Optimizer::findSeedPointsParallel()
// {
//     int K = 1;
//     vector<NNSearchF*> nabos;
//     vector<MatrixXf> mats;
//     for(int i=0;i<clouds.size();i++)
//     {
//         MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//         for(int i=0;i<cloud->points.size();i++)
//         {
//             auto pt = cloud->points[i];
//             M(0,i) = pt.x;
//             M(1,i) = pt.y;
//             M(2,i) = pt.z;
//         }
//         NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//         mats.push_back(M);
//         nabos.push_back(nns);
//     }
//     
//     vector<double> error_avg(clouds.size(),1.0/0.0);
//     vector<double> error_max(clouds.size(),1.0/0.0);
//     double max_error = -1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     TIC();
//     #pragma omp parallel
//     #pragma omp for
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         VectorXf q = VectorXf::Zero(3);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             q(0) = out[0];
//             q(1) = out[1];
//             q(2) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//         
//         VectorXi indices(K);
//         VectorXf dists2(K);
//         nabos[j]->knn(q, indices, dists2, K,0.1,NNSearchF::SORT_RESULTS);
//         double distance = sqrt(dists2(0));
//         if(maximum<distance)
//             maximum=distance;
//         average+=distance;
//         counter++;
//         }
//         average=average/counter;
//         error_avg[j]=average;
//         error_max[j]=maximum;
//     }
//     TOC();
//     cout<<"Errors: "<<endl;
//     for(int i=0;i<error_avg.size();i++)
//     {
//         cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
//     }
//     // delete nns;
// }

void Optimizer::discreteCombintorialOptimizer()
{
    int K = 1;
    MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    double min_error = 1e9;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    vector<double> trans(6),flange_trans(6);
    int iter_min = -2;
    int iter_max = 2;
    TIC();
    int iterations = 0;
    for(int xf=iter_min;xf<=iter_max;xf+=2)
        for(int yf=iter_min;yf<=iter_max;yf+=2)
            for(int zf=iter_min;zf<=iter_max;zf+=2)
                for(int xo=iter_min;xo<=iter_max;xo+=2)
                    for(int yo=iter_min;yo<=iter_max;yo+=2)
                        for(int zo=iter_min;zo<=iter_max;zo+=2)
                        {
                            double err = 0.0;
                            flange_transformation[0]+=xf/1000.0;
                            flange_transformation[1]+=yf/1000.0;
                            flange_transformation[2]+=zf/1000.0;
                            transformation[0]+=xo/1000.0;
                            transformation[1]+=yo/1000.0;
                            transformation[2]+=zo/1000.0;
                            TIC();
                            // #pragma omp parallel
                            // #pragma omp for
                            for(int j=0;j<clouds.size();j++)
                            {
                                float average = 0.0;
                                float maximum = -1e9;
                                int counter = 0;
                                Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
                                Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
                                Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
                                world_T_object = world_T_object.inverse();
                                MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
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
                                    N(0,i) = out[0];
                                    N(1,i) = out[1];
                                    N(2,i) = out[2];
#else
                                    pts(0,0)=point.x;
                                    pts(0,1)=point.y;
                                    pts(0,2)=point.z;
                                    pts=apply_transformation(pts,transformation);
                                    pts=apply_transformation(pts,world_T_object);
                                    N(0,i) = pts(0,0);
                                    N(1,i) = pts(0,1);
                                    N(2,i) = pts(0,2);
#endif
                                    counter++;
                                }
                                MatrixXi indices;
                                MatrixXf dists2;
                                indices.resize(1, N.cols());
                                dists2.resize(1, N.cols());
                                nns->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);

                                for(int i=0;i<counter;i++)
                                {
                                    double distance = sqrt(dists2(0,i));
                                    if(maximum<distance)
                                        maximum=distance;
                                    average+=distance;
                                }
                                average=average/counter;
                                err = err + average*0.5 + maximum*0.5;
                            }
                            if(err<min_error)
                            {
                                min_error = err;
                                flange_trans = flange_transformation;
                                trans = transformation;
                            }
                            flange_transformation[0]-=xf/1000.0;
                            flange_transformation[1]-=yf/1000.0;
                            flange_transformation[2]-=zf/1000.0;
                            transformation[0]-=xo/1000.0;
                            transformation[1]-=yo/1000.0;
                            transformation[2]-=zo/1000.0;
                            iterations++;
                        }
    TOC();
    cout<<"Iterations: "<<iterations<<endl;
    cout<<"Minimum Error: "<<min_error<<endl;
    cout<<"Flange Transformation"<<endl;
    for(auto x:flange_trans)
        cout<<x<<" ";
    cout<<"Object Transformation"<<endl;
    for(auto x:trans)
        cout<<x<<" ";
}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    Optimizer opti = Optimizer(string(argv[1]));
    opti.getInputs();
    // opti.findSeedPoints();
    opti.discreteCombintorialOptimizer();
	return 0;
}
//     int K = 1;
//     cout<<"Starting Optimization. "<<endl;
//     MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//     for(int i=0;i<cloud->points.size();i++)
//     {
//         auto pt = cloud->points[i];
//         M(0,i) = pt.x;
//         M(1,i) = pt.y;
//         M(2,i) = pt.z;
//     }
//     NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//     double min_error = 1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     vector<double> trans(6),flange_trans(6);
//     int iter_min = -2;
//     int iter_max = 2;
//     // for(int xf=iter_min;xf<=iter_max;xf+=2)
//     // for(int yf=iter_min;yf<=iter_max;yf+=2)
//     // for(int zf=iter_min;zf<=iter_max;zf+=2)
//     // for(int xo=iter_min;xo<=iter_max;xo+=2)
//     // for(int yo=iter_min;yo<=iter_max;yo+=2)
//     // for(int zo=iter_min;zo<=iter_max;zo+=2)
//     // {
//     double err = 0.0;
//     // flange_transformation[0]+=xf/1000.0;
//     // flange_transformation[1]+=yf/1000.0;
//     // flange_transformation[2]+=zf/1000.0;
//     // transformation[0]+=xo/1000.0;
//     // transformation[1]+=yo/1000.0;
//     // transformation[2]+=zo/1000.0;
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             N(0,i) = out[0];
//             N(1,i) = out[1];
//             N(2,i) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//             counter++;
//         }
//         MatrixXi indices;
//         MatrixXf dists2;
//         indices.resize(1, N.cols());
//         dists2.resize(1, N.cols());
//         nns->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);        for(int i=0;i<counter;i++)
//         {
//             double distance = sqrt(dists2(0,i));
//             if(maximum<distance)
//                 maximum=distance;
//             average+=distance;
//         }
//         average=average/counter;
//         // err = err + average*0.5 + maximum*0.5;
//     }
//     // if(err<min_error)
//     // {
//     //     min_error = err;
//     //     flange_trans = flange_transformation;
//     //     trans = transformation;
//     // }
//     // flange_transformation[0]-=xf/1000.0;
//     // flange_transformation[1]-=yf/1000.0;
//     // flange_transformation[2]-=zf/1000.0;
//     // transformation[0]-=xo/1000.0;
//     // transformation[1]-=yo/1000.0;
//     // transformation[2]-=zo/1000.0;
//     // }
//     cout<<"Min Error: "<<min_error<<endl;
//     cout<<"Flange Transformation"<<endl;
//     for(auto x:flange_trans)
//         cout<<x<<" ";
//     cout<<"Object Transformation"<<endl;
//     for(auto x:trans)
//         cout<<x<<" ";
//     delete nns;
//

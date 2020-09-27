/***********************************************/
//STANDARD HEADERS
/************************************************/
#include <iostream>
#include <cmath>
#include <vector>
#include <utility>
#include <chrono>
#include <unordered_map> 
#include <queue>
#include <fstream>
#include <thread>
#include <ctime>

#include "pclviewer.h"
#include "ui_pclviewer.h"
/*********************************************/
//PCL HEADERS
/**********************************************/
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <omp.h>
#include "nabo/nabo.h"
#include "algorithms.hpp"

using namespace std;
using namespace Eigen;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;
using namespace Nabo;
using namespace pcl;


namespace PointCloudProcessing
{
    
    constexpr int degree(double radian){return int((radian*180)/3.14159);};
    constexpr double magnitude(float normal[3]){return normal[0]*normal[0]+normal[1]*normal[1]+normal[2]*normal[2];};
    constexpr int angle(float normal[3]){return degree(acos(-normal[2]/magnitude(normal)));};
    template <typename T> constexpr int sgn(T x) {
        return (T(0) < x) - (x < T(0));
    }

    constexpr double k_AngleMin = 0;
    constexpr double k_AngleMax = 45;
    constexpr double k_ZMin = 0.20;
    constexpr double k_ZMax = 1.0;


    void classifyPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_classified)
    {
        PointCloud<PointXYZ>::Ptr cloud_bw (new PointCloud<PointXYZ>);
        for(int i=0;i<cloud->points.size();i++)
        {
            PointXYZ ptxyz;
            PointXYZRGB ptxyzrgb = cloud->points[i];
            ptxyz.x=ptxyzrgb.x;
            ptxyz.y=ptxyzrgb.y;
            ptxyz.z=ptxyzrgb.z*sgn(ptxyzrgb.z);
            cloud_bw->points.push_back(ptxyz);
        }
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_bw);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (0.005);
        ne.setViewPoint(0,0,0);
        ne.compute (*normals); 

        for(int i=0;i<normals->points.size();i++)
        {
            Normal ptn = normals->points[i];
            PointXYZ pt = cloud_bw->points[i];
            int angle_z = angle(ptn.normal);
            PointXYZRGB pt_new;
            pt_new.x = pt.x;
            pt_new.y = pt.y;
            pt_new.z = pt.z;
            pt_new.g = 0;
            pt_new.r = 255;
            pt_new.b = 0;
            if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax&&pt_new.z>=k_ZMin&&pt_new.z<=k_ZMax)
            {
                pt_new.b = 0;
                pt_new.r = 0;
                pt_new.g = 255;
            }
            cloud_classified->points.push_back(pt_new);
        }
    }
    
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered)
    {
        PointCloud<PointXYZ>::Ptr cloud_bw (new PointCloud<PointXYZ>);
        for(int i=0;i<cloud->points.size();i++)
        {
            PointXYZ ptxyz;
            PointXYZRGB ptxyzrgb = cloud->points[i];
            ptxyz.x=ptxyzrgb.x;
            ptxyz.y=ptxyzrgb.y;
            ptxyz.z=ptxyzrgb.z*sgn(ptxyzrgb.z);
            cloud_bw->points.push_back(ptxyz);
        }
        pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud (cloud_bw);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
        ne.setSearchMethod (tree);
        pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
        ne.setRadiusSearch (0.005);
        ne.setViewPoint(0,0,0);
        ne.compute (*normals); 

        for(int i=0;i<normals->points.size();i++)
        {
            Normal ptn = normals->points[i];
            PointXYZRGB pt = cloud->points[i];
            int angle_z = angle(ptn.normal);
            PointXYZRGB pt_new;
            pt_new.x = pt.x;
            pt_new.y = pt.y;
            pt_new.z = pt.z*sgn(pt.z);
            pt_new.g = pt.g;
            pt_new.r = pt.r;
            pt_new.b = pt.b;
            if(angle_z>=k_AngleMin&&angle_z<=k_AngleMax&&pt_new.z>=k_ZMin&&pt_new.z<=k_ZMax)
            {
                cloud_filtered->points.push_back(pt_new);
            }
        }
    }
};

void PCLViewer::findSeedPoints()
{
    int K = 1;

    MatrixXf M = MatrixXf::Zero(3, cloud_->points.size());
    for(int i=0;i<cloud_->points.size();i++)
    {
        auto pt = cloud_->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);

    vector<double> error_avg(clouds_.size(),1.0/0.0);
    vector<double> error_max(clouds_.size(),1.0/0.0);
    double max_error = -1e9;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    // #pragma omp parallel
    // #pragma omp for
    for(int j=0;j<clouds_.size();j++)
    {
        if(selected_clouds_[j]==false)
        {
            continue;
        }
        float average = 0.0;
        float maximum = -1e9;
        int counter = 0;
        Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation_);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation_);
        Eigen::MatrixXd transformation = inverse_kinematics_[j]*cam_T_flange;
        world_T_object = world_T_object.inverse();
        MatrixXf N = MatrixXf::Zero(3, cloud_downsampled_[j]->points.size());
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        Eigen::Affine3d transW;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                transW(a,b) = world_T_object(a,b);
        for(int i=0;i<cloud_downsampled_[j]->points.size();i++)
        {
            auto point = cloud_downsampled_[j]->points[i];
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
        nns->knn(N, indices, dists2, 1, 0, NNSearchF::SORT_RESULTS);
        for(int i=0;i<counter;i++)
        {
            double distance = sqrt(dists2(0,i));
            if(maximum<distance)
                maximum=distance;
            average+=distance;
        }
        average=average/counter;
        std::cout<<average<<" "<<counter<<endl;
        cout<<"Counted "<<counter<<" points."<<endl;
        // cout<<"Max Error : "<<max_error/1000.0<<endl;
        error_avg[j]=average;
        error_max[j]=maximum;
    }

    cout<<"Errors: "<<endl;
    for(int i=0;i<error_avg.size();i++)
    {
        cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
    }
}

void PCLViewer::showErrorsInPoints()
{
    int K = 1;
    MatrixXf M = MatrixXf::Zero(3, cloud_->points.size());
    for(int i=0;i<cloud_->points.size();i++)
    {
        auto pt = cloud_->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    vector<PointCloudT::Ptr> processed;
    for(int j=0;j<clouds_.size();j++)
    {
        PointCloudT::Ptr cloud_temp(new PointCloudT);
        if(selected_clouds_[j]==false)
        {
            processed.push_back(cloud_temp);
            continue;
        }
        float average = 0.0;
        float maximum = -1e9;
        int counter = 0;
        Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation_);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation_);
        Eigen::MatrixXd transformation = inverse_kinematics_[j]*cam_T_flange;
        world_T_object = world_T_object.inverse();
        MatrixXf N = MatrixXf::Zero(3, clouds_[j]->points.size());
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        Eigen::Affine3d transW;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                transW(a,b) = world_T_object(a,b);
        for(int i=0;i<clouds_[j]->points.size();i++)
        {
            auto point = clouds_[j]->points[i];
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
        nns->knn(N, indices, dists2, 1, 0, NNSearchF::SORT_RESULTS);
        for(int i=0;i<counter;i++)
        {
            double distance = sqrt(dists2(0,i));
            auto point = clouds_[j]->points[i];
            point.r = 0;
            point.g = 0;
            point.b = 0;
            if(distance*1000.0<2.5)
                point.g=255;
            else if(distance*1000.0<5.0)
                point.b=255;
            else if(distance*1000.0<10.0)
            {
                point.r = 100;
                point.g = 100;
            }
            else
                point.r=255;
            cloud_temp->points.push_back(point);
        }
        processed.push_back(cloud_temp);
        cout<<"Counted "<<counter<<" points."<<endl;
    }
    updateClouds(processed);
}


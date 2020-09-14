#pragma once

#include <iostream>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;

namespace Helpers
{
    inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
    {
        Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
        Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
        data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
        data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
        Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
        Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
        transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
        return transformed_data_mat.transpose();
    }

    template<typename PointT> void transformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud_input,typename pcl::PointCloud<PointT>::Ptr cloud_output,Eigen::MatrixXd& a_T_b)
    {
        for(auto point:cloud_input->points)
        {
            Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
            auto pt = point;
            pt.x=pts_trans(0,0);
            pt.y=pts_trans(0,1);
            pt.z=pts_trans(0,2);
            cloud_output->points.push_back(pt);
        }
    }
    template<typename PointT> void transformPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud_input,Eigen::MatrixXd& a_T_b)
    {
        for(auto& point:cloud_input->points)
        {
            Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            Eigen::MatrixXd pts_trans=apply_transformation(pts,a_T_b);
            point.x=pts_trans(0,0);
            point.y=pts_trans(0,1);
            point.z=pts_trans(0,2);
        }
    }
    std::string validate_seq(std::string seq)
    {
        if(seq =="")
            seq = "ZYX";	
        bool invalid_flag = false;
        if(seq.size()!=3)
        {
            invalid_flag = true;
        }
        for (int i =0;i<3;++i)
            if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z' && seq[i]!='x' && seq[i]!='y' && seq[i]!='z')
            {
                invalid_flag = true; 
                break;
            }
        if(invalid_flag)
        {
            std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
            std::terminate();		
        }
        return seq;
    }

    //Default : ZYX
    Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq="")
    {
        seq = validate_seq(seq);
        Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
        for (int i=0; i<3; ++i)
        {
            if(seq[i]=='X' || seq[i]=='x')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitX());
            else if(seq[i]=='Y' || seq[i]=='y')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitY());			
            else if(seq[i]=='Z' || seq[i]=='z')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitZ());					
        }
        return rot_mat; 
    }

    Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static)
    {
        Eigen::MatrixXd a_T_b = Eigen::MatrixXd::Identity(4,4);
        a_T_b(0,3) = a_T_b_static[0];        
        a_T_b(1,3) = a_T_b_static[1];
        a_T_b(2,3) = a_T_b_static[2];
        Eigen::MatrixXd eul_ang(1,3);
        eul_ang<<a_T_b_static[3],a_T_b_static[4],a_T_b_static[5];
        a_T_b.block(0,0,3,3) = eul2rot(eul_ang);
        return a_T_b;
    }

    constexpr double degreeToRadian(int angle)
    {
        return angle*3.141592/180;
    }
    
    void drawGrid(pcl::visualization::PCLVisualizer::Ptr viewer,double xmin=-3.0,double xmax=3.0,double ymin=-3.0,double ymax=3.0,double gridsize=0.5)
    {
        for(double x=xmin;x<=xmax;x+=gridsize)
        {
            pcl::PointXYZ pt1,pt2;
            pt1.x = x;
            pt1.y = ymax;
            pt1.z = 0;
            pt2.x = x;
            pt2.y = ymin;
            pt2.z = 0;
            viewer->addLine<pcl::PointXYZ> (pt1,pt2,"linex"+to_string(x));
        }
        for(double y=ymin;y<=ymax;y+=gridsize)
        {
            pcl::PointXYZ pt1,pt2;
            pt1.x = xmin;
            pt1.y = y;
            pt1.z = 0;
            pt2.x = xmax;
            pt2.y = y;
            pt2.z = 0;
            viewer->addLine<pcl::PointXYZ> (pt1,pt2,"liney"+to_string(y));
        }
    }
};

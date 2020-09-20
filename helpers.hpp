#pragma once

#include <iostream>
#include <algorithm>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

namespace TransformationUtilities
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
    
	constexpr double radTodeg(double rad)
	{
		return (rad/3.14159)*180;
	}
    
    Eigen::Affine3f getAffineMatrix(Eigen::MatrixXd &transformation)
    {
        Eigen::Affine3f temp;
        for(int k=0;k<3;k++)
            for(int j=0;j<4;j++)
                temp(k,j) = transformation(k,j);
        return temp;
    }

};

namespace InputUtilities
{

    void readPointCloud(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
    {
        vector<string> result; 
        boost::split(result,filename, boost::is_any_of("."));
        string type = result[result.size()-1];
        std::for_each(type.begin(), type.end(), [](char & c){
                c = ::tolower(c);
                });
        if(type=="pcd")
        {
            if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *cloud) == -1) //* load the file
            {
                PCL_ERROR ("Couldn't read file for base. \n");
                throw;
            }
        }
        else if(type=="ply")
        {
            pcl::PLYReader reader;
            reader.read(filename, *cloud);
        }
        else if(type=="xyz")
        {
            ifstream file(filename);
            string metrics;
            getline(file,metrics);
            double z_min=1e9,z_max=-1e9;
            //TODO: Read pointcloud here.
            while(getline(file,metrics)&&metrics.size())
            {
                vector<string> result; 
                boost::split(result,metrics, boost::is_any_of(" "));
                string temp = result[1];
                vector<string> coords;
                boost::split(coords, temp, boost::is_any_of(","));
                pcl::PointXYZRGB pt;
                pt.x = stof(coords[0])/1000.0;
                pt.y = stof(coords[1])/1000.0;
                pt.z = stof(coords[2])/1000.0;
                pt.r = 0; 
                pt.g = 0;
                pt.b = 0;
                if(pt.z<z_min)
                    z_min = pt.z;
                if(pt.z>z_max)
                    z_max=pt.z;
                cloud->points.push_back(pt);
            }
            //Color adjustment based on depth.
            for(auto &pt:cloud->points)
            {
                double scale = (pt.z-z_min)/(z_max-z_min);
                pt.r = pt.g = pt.b = int(255.0*scale);
            }
        }
        else
        {
            cout<<"Format not supported."<<endl;
            throw;
        }
    }

    vector<MatrixXd> readTransformations(string filename,bool affine=false)
    {
		cout<<"Inside transformation reader"<<endl;
        vector<MatrixXd> transformations;
        ifstream file(filename);
        MatrixXd mat=MatrixXd::Zero(4,4);
        if(affine==false)
        {
            while(true)
            {
                string line;
                for(int i=0;i<4;i++)
                {
                    getline(file,line);
                    if(line.size()==0)
                        break;
                    vector<string> v;
                    split(v,line,boost::is_any_of(","));
                    for(int j=0;j<v.size();j++)
                        mat(i,j)=stof(v[j]);
                }
                transformations.push_back(mat);
            }
        }
        else
        {
            string trans;
            while(getline(file,trans)&&trans.size())
            {
                vector<string> coords_str;
                boost::split(coords_str, trans, boost::is_any_of(","));
                vector<double> coords;
                for(int i=0;i<coords_str.size();i++)
                    if(i<3)
                        coords.push_back(stof(coords_str[i])/1000.0);
                    else
                        coords.push_back(stof(coords_str[i]));
                auto mat = TransformationUtilities::vectorToTransformationMatrix(coords);
                transformations.push_back(mat);
            }
        }
        return transformations;
    }
};

namespace InterfaceUtilities 
{
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
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.3, "linex"+to_string(x));
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
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1.3, "liney"+to_string(y));
        }
    }

    void addClouds(vector<PointCloudT::Ptr>& clouds,vector<PointCloudT::Ptr>& cloud_outputs,vector<MatrixXd>& iks,vector<double> & flange_transformation,pcl::visualization::PCLVisualizer::Ptr viewer)
    {
        cout<<"Inside adding cloud"<<endl;
		cout<<clouds.size()<<" "<<iks.size()<<endl;
        if(clouds.size()>iks.size())
        {
            cout<<"Size of the iks provided is not matching the size of the pointclouds. Check the input files."<<endl;
            throw;
        }
        for(int i=0;i<clouds.size();i++)
        {
            cloud_outputs[i]->clear();
            Eigen::MatrixXd cam_T_flange = TransformationUtilities::vectorToTransformationMatrix(flange_transformation);
            Eigen::MatrixXd transformation = iks[i]*cam_T_flange;
            TransformationUtilities::transformPointCloud<pcl::PointXYZRGB>(clouds[i],cloud_outputs[i],transformation);
            viewer->addPointCloud (cloud_outputs[i], "cloud"+to_string(i+1));
            cout<<"Adding cloud: "<<i<<endl;
        }
    }

    void updateClouds(vector<PointCloudT::Ptr>& clouds,vector<PointCloudT::Ptr>& cloud_outputs,vector<MatrixXd>& iks,vector<double> & flange_transformation,pcl::visualization::PCLVisualizer::Ptr viewer,vector<bool>& selected)
    {
        if(clouds.size()>iks.size())
        {
            throw "Size Mismatch Error.\n";
        }
        for(int i=0;i<clouds.size();i++)
        {
            cloud_outputs[i]->clear();
            Eigen::MatrixXd cam_T_flange = TransformationUtilities::vectorToTransformationMatrix(flange_transformation);
            Eigen::MatrixXd transformation = iks[i]*cam_T_flange;
            if(selected[i])
            {
                TransformationUtilities::transformPointCloud<pcl::PointXYZRGB>(clouds[i],cloud_outputs[i],transformation);
            }
            viewer->updatePointCloud (cloud_outputs[i], "cloud"+to_string(i+1));
        }
    }

    string getErrorMetrics(vector<PointCloudT::Ptr> clouds, PointCloudT::Ptr object)
    {
        string html = "<table><tr><th>Cloud Id</th><th>Avg Error</th></tr>";
        for(int i=0;i<clouds.size();i++)
        {
            html+="<tr><td>"+to_string(i)+"</td><td>INF</td></tr>";
        }
        html+="</table>";
        return html;
    }
    
    void addObjectToSpace(PointCloudT::Ptr cloud,PointCloudT::Ptr output,vector<double>& transformation,pcl::visualization::PCLVisualizer::Ptr viewer)
    {
        output->clear();
        Eigen::MatrixXd part_T_base = TransformationUtilities::vectorToTransformationMatrix(transformation);
        TransformationUtilities::transformPointCloud<pcl::PointXYZRGB>(cloud,output,part_T_base);
        viewer->addPointCloud (output, "object");
    }

    void updateObjectToSpace(PointCloudT::Ptr cloud,PointCloudT::Ptr output,vector<double>& transformation,pcl::visualization::PCLVisualizer::Ptr viewer,bool selected=true)
    {
		output->clear();
		Eigen::MatrixXd part_T_base = TransformationUtilities::vectorToTransformationMatrix(transformation);
        cout<<"object_T_base Transformation: "<<endl;
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
				cout<<part_T_base(i,j)<<" ";
			cout<<endl;
		}
		if(selected)
		{
            TransformationUtilities::transformPointCloud<pcl::PointXYZRGB>(cloud,output,part_T_base);
        }
        viewer->updatePointCloud (output, "object");
    }
};



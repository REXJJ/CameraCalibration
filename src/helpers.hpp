#pragma once

#include <iostream>
#include <algorithm>
#include <chrono>
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

#if defined(__SSE2__)
#include <xmmintrin.h>
#endif

#if defined(__AVX__)
#include <immintrin.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

/** A helper struct to apply an SO3 or SE3 transform to a 3D point.
 * Supports single and double precision transform matrices. */
template<typename Scalar>
struct Transformer
{
    const Eigen::Matrix<Scalar, 4, 4>& tf;

    /** Construct a transformer object.
     * The transform matrix is captured by const reference. Make sure that it does not go out of scope before this
     * object does. */
    Transformer (const Eigen::Matrix<Scalar, 4, 4>& transform) : tf (transform) { };

    /** Apply SO3 transform (top-left corner of the transform matrix).
     * \param[in] src input 3D point (pointer to 3 floats)
     * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 0. */
    void so3 (const float* src, float* tgt) const
    {
        const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
        tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2]);
        tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2]);
        tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2]);
        tgt[3] = 0;
    }

    /** Apply SE3 transform.
     * \param[in] src input 3D point (pointer to 3 floats)
     * \param[out] tgt output 3D point (pointer to 4 floats), can be the same as input. The fourth element is set to 1. */
    void se3 (const float* src, float* tgt) const
    {
        const Scalar p[3] = { src[0], src[1], src[2] };  // need this when src == tgt
        tgt[0] = static_cast<float> (tf (0, 0) * p[0] + tf (0, 1) * p[1] + tf (0, 2) * p[2] + tf (0, 3));
        tgt[1] = static_cast<float> (tf (1, 0) * p[0] + tf (1, 1) * p[1] + tf (1, 2) * p[2] + tf (1, 3));
        tgt[2] = static_cast<float> (tf (2, 0) * p[0] + tf (2, 1) * p[1] + tf (2, 2) * p[2] + tf (2, 3));
        tgt[3] = 1;
    }
};

#if defined(__SSE2__)

/** Optimized version for single-precision transforms using SSE2 intrinsics. */
template<>
struct Transformer<float>
{
    /// Columns of the transform matrix stored in XMM registers.
    __m128 c[4];

    Transformer(const Eigen::Matrix4f& tf)
    {
        for (std::size_t i = 0; i < 4; ++i)
            c[i] = _mm_load_ps (tf.col (i).data ());
    }

    void so3 (const float* src, float* tgt) const
    {
        __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
        __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
        __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
        _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, p2)));
    }

    void se3 (const float* src, float* tgt) const
    {
        __m128 p0 = _mm_mul_ps (_mm_load_ps1 (&src[0]), c[0]);
        __m128 p1 = _mm_mul_ps (_mm_load_ps1 (&src[1]), c[1]);
        __m128 p2 = _mm_mul_ps (_mm_load_ps1 (&src[2]), c[2]);
        _mm_store_ps (tgt, _mm_add_ps(p0, _mm_add_ps(p1, _mm_add_ps(p2, c[3]))));
    }
};

#if !defined(__AVX__)

/** Optimized version for double-precision transform using SSE2 intrinsics. */
template<>
struct Transformer<double>
{
    /// Columns of the transform matrix stored in XMM registers.
    __m128d c[4][2];

    Transformer(const Eigen::Matrix4d& tf)
    {
        for (std::size_t i = 0; i < 4; ++i)
        {
            c[i][0] = _mm_load_pd (tf.col (i).data () + 0);
            c[i][1] = _mm_load_pd (tf.col (i).data () + 2);
        }
    }

    void so3 (const float* src, float* tgt) const
    {
        __m128d xx = _mm_cvtps_pd (_mm_load_ps1 (&src[0]));
        __m128d p0 = _mm_mul_pd (xx, c[0][0]);
        __m128d p1 = _mm_mul_pd (xx, c[0][1]);

        for (std::size_t i = 1; i < 3; ++i)
        {
            __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
            p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
            p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
        }

        _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
    }

    void se3 (const float* src, float* tgt) const
    {
        __m128d p0 = c[3][0];
        __m128d p1 = c[3][1];

        for (std::size_t i = 0; i < 3; ++i)
        {
            __m128d vv = _mm_cvtps_pd (_mm_load_ps1 (&src[i]));
            p0 = _mm_add_pd (_mm_mul_pd (vv, c[i][0]), p0);
            p1 = _mm_add_pd (_mm_mul_pd (vv, c[i][1]), p1);
        }

        _mm_store_ps (tgt, _mm_movelh_ps (_mm_cvtpd_ps (p0), _mm_cvtpd_ps (p1)));
    }
};

#else

/** Optimized version for double-precision transform using AVX intrinsics. */
template<>
struct Transformer<double>
{
    __m256d c[4];

    Transformer(const Eigen::Matrix4d& tf)
    {
        for (std::size_t i = 0; i < 4; ++i)
            c[i] = _mm256_load_pd (tf.col (i).data ());
    }

    void so3 (const float* src, float* tgt) const
    {
        __m256d p0 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[0])), c[0]);
        __m256d p1 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[1])), c[1]);
        __m256d p2 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[2])), c[2]);
        _mm_store_ps (tgt, _mm256_cvtpd_ps (_mm256_add_pd(p0, _mm256_add_pd(p1, p2))));
    }

    void se3 (const float* src, float* tgt) const
    {
        __m256d p0 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[0])), c[0]);
        __m256d p1 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[1])), c[1]);
        __m256d p2 = _mm256_mul_pd (_mm256_cvtps_pd (_mm_load_ps1 (&src[2])), c[2]);
        _mm_store_ps (tgt, _mm256_cvtpd_ps (_mm256_add_pd(p0, _mm256_add_pd(p1, _mm256_add_pd(p2, c[3])))));
    }
};

#endif // !defined(__AVX__)
#endif // defined(__SSE2__)

using namespace std;
using namespace Eigen;

inline void apply_transformation_optimized(float* data,float* output,Eigen::Affine3d& transformation)
{
    Transformer<double> tf(transformation.matrix());
    tf.se3(data,output);
}

/*****************************************************************/
//Color Definitions
/****************************************************************/
#define _NORMAL_    "\x1b[0m"
#define _BLACK_     "\x1b[30;47m"
#define _RED_       "\x1b[31;40m"
#define _GREEN_     "\x1b[32;40m"
#define _YELLOW_    "\x1b[33;40m"
#define _BLUE_      "\x1b[34;40m"
#define _MAGENTA_   "\x1b[35;40m"
#define _CYAN_      "\x1b[36;40m"
#define _WHITE_     "\x1b[37;40m"
#define _BRED_      "\x1b[1;31;40m"
#define _BGREEN_    "\x1b[1;32;40m"
#define _BYELLOW_   "\x1b[1;33;40m"
#define _BBLUE_     "\x1b[1;34;40m"
#define _BMAGENTA_  "\x1b[1;35;40m"
#define _BCYAN_     "\x1b[1;36;40m"
#define _BWHITE_    "\x1b[1;37;40m"
/******************************************************************/
//Print Utilities for debugging.
/*******************************************************************/
#define dbg_color(...) do { printf(__VA_ARGS__); } while(0)

#define TIC() std::chrono::high_resolution_clock::time_point END_TIME,START_TIME=std::chrono::high_resolution_clock::now()

#define RESET() START_TIME=std::chrono::high_resolution_clock::now()

#define TOC()                                                   \
    do                                                              \
{                                                               \
    dbg_color(_GREEN_);                                         \
    END_TIME=std::chrono::high_resolution_clock::now();         \
    std::cout << " Time elapsed in "<<__func__<<"() at line "<<__LINE__<<": "         \
    << std::chrono::duration_cast<std::chrono::microseconds>(END_TIME - START_TIME).count() << " microseconds.\n"; \
    dbg_color(_NORMAL_);                                        \
}while(0)


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

    inline Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static)
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

    Eigen::MatrixXd rot2eul(Eigen::Matrix3d rot_mat, std::string seq)
    {
        seq = validate_seq(seq);
        int rot_idx[3];
        for (int i=0; i<3; ++i)
        {
            if(seq[i]=='X' || seq[i]=='x')
                rot_idx[i] = 0;
            else if(seq[i]=='Y' || seq[i]=='y')
                rot_idx[i] = 1;
            else if(seq[i]=='Z' || seq[i]=='z')
                rot_idx[i] = 2;
        }	
        Eigen::MatrixXd eul_angles(1,3);
        Eigen::Vector3d eul_angles_vec;
        eul_angles_vec = rot_mat.eulerAngles(rot_idx[0], rot_idx[1], rot_idx[2]);
        eul_angles(0,0) = eul_angles_vec[0];
        eul_angles(0,1) = eul_angles_vec[1];
        eul_angles(0,2) = eul_angles_vec[2];
        return eul_angles;
    }

};

namespace InputUtilities
{

    void readPointCloud(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,string metric="m")
    {
        double scale = 1.0;
        if(metric=="mm")
            scale=1000.0;
        else if(metric=="cm")
            scale=100.0;
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
            if(metric!="m")
                for(auto &pt:cloud->points)
                {
                    pt.x=pt.x/scale;
                    pt.y=pt.y/scale;
                    pt.z=pt.z/scale;
                }
        }
        else if(type=="ply")
        {
            pcl::PLYReader reader;
            reader.read(filename, *cloud);
            if(metric!="m")
                for(auto &pt:cloud->points)
                {
                    pt.x=pt.x/scale;
                    pt.y=pt.y/scale;
                    pt.z=pt.z/scale;
                }

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
                pt.x = stof(coords[0])/scale;
                pt.y = stof(coords[1])/scale;
                pt.z = stof(coords[2])/scale;
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
        string metric="mm";
        cout<<"Inside transformation reader"<<endl;
        vector<MatrixXd> transformations;
        ifstream file(filename);
        MatrixXd mat=MatrixXd::Zero(4,4);
        double scale = 1.0;
        if(metric=="mm")
            scale=1000.0;
        else if(metric=="cm")
            scale=100.0;
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
                cout<<trans<<endl;
                vector<string> coords_str;
                boost::split(coords_str, trans, boost::is_any_of(","));
                vector<double> coords;
                for(int i=0;i<coords_str.size();i++)
                    if(i<3)
                        coords.push_back(stof(coords_str[i])/scale);
                    else
                        coords.push_back(stof(coords_str[i]));

                for(auto x:coords)
                    cout<<x<<" ";
                cout<<endl;
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



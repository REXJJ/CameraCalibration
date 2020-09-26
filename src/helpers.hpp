#ifndef HELPERS_HPP
#define HELPERS_HPP

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
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>

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
    inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat);
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
    std::string validate_seq(std::string seq);
    //Default : ZYX
    Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq="");

    inline Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static);
    
    constexpr double degreeToRadian(int angle)
    {
        return angle*3.141592/180;
    }

    constexpr double radTodeg(double rad)
    {
        return (rad/3.14159)*180;
    }

    Eigen::Affine3f getAffineMatrix(Eigen::MatrixXd &transformation);
    Eigen::MatrixXd rot2eul(Eigen::Matrix3d rot_mat, std::string seq);
};

namespace InputUtilities
{
    void readPointCloud(string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,string metric="m");
    vector<MatrixXd> readTransformations(string filename,bool affine=false);
    vector<double> getTransVector(boost::property_tree::ptree &pt,string s);
};

namespace InterfaceUtilities 
{
    void drawGrid(pcl::visualization::PCLVisualizer::Ptr viewer,double xmin=-3.0,double xmax=3.0,double ymin=-3.0,double ymax=3.0,double gridsize=0.5);
    void addClouds(vector<PointCloudT::Ptr>& clouds,vector<PointCloudT::Ptr>& cloud_outputs,vector<MatrixXd>& iks,vector<double> & flange_transformation,pcl::visualization::PCLVisualizer::Ptr viewer);
    void addObjectToSpace(PointCloudT::Ptr cloud,PointCloudT::Ptr output,vector<double>& transformation,pcl::visualization::PCLVisualizer::Ptr viewer);
    void updateObjectToSpace(PointCloudT::Ptr cloud,PointCloudT::Ptr output,vector<double>& transformation,pcl::visualization::PCLVisualizer::Ptr viewer,bool selected=true);
};
#endif


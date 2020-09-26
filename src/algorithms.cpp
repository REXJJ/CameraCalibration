#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <pcl/io/ply_io.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <omp.h>
#include "nabo/nabo.h"

using namespace std;
using namespace Eigen;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;
using namespace Nabo;

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


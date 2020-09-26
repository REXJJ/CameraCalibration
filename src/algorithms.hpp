#pragma once

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
/*********************************************/
//OTHER HEADERS
/**********************************************/
#include <Eigen/Dense>
#include <Eigen/Core>
#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "algorithms.hpp"
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <omp.h>
#include "nabo/nabo.h"

using namespace std;
using namespace Eigen;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;
using namespace Nabo;
using namespace pcl;


namespace PointCloudProcessing
{
    template <typename PointT> static pcl::PointCloud<PointT> downsample(pcl::PointCloud<PointT> cloud,double leaf)
    {
        pcl::VoxelGrid<PointT> sor;
        pcl::PointCloud<PointT> cloud_filtered;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.setInputCloud (cloud.makeShared());
        sor.filter (cloud_filtered);
        return cloud_filtered;
    }

    void classifyPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_classified);
    void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered);
};

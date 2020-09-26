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

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui_ (new Ui::PCLViewer)
{
    //Setting up the private members. 
    ui_->setupUi (this);
    this->setWindowTitle ("PCL viewer");
    cloud_.reset (new PointCloudT);
    cloud_output_.reset (new PointCloudT);
    transformation_ = std::vector<double>(6,0);
    flange_transformation_ = std::vector<double>(6,0);
    transformation_initial_ = std::vector<double>(6,0);
    flange_transformation_initial_ = std::vector<double>(6,0);
    translation_resolution_ = 1.0;
    rotation_resolution_ = 2.0;
    calculate_error_= false;
    apply_svd_ = false;
    algorithm_ = 0;
    z_threshold_ = 100;
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    // if(QCoreApplication::arguments().size()!=2)
    // {
    //     cout<<"Usage: ./pcl_visualizer <config file path> "<<endl;
    //     exit(-1);
    // }
    getInputs();
    setupViz();
    setupInterface();
    addWidgets();
}

PCLViewer::~PCLViewer ()
{
    delete ui_;
}
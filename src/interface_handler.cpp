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

void PCLViewer::getInputs()
{
    // Reading the pointclouds, sensor scans and the transformations from files using XML config file.
    ifstream file;
    string config_file = "/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/CameraCalibration/config/config.xml";
    QStringList config_loc = QCoreApplication::arguments();
    std::cout<<"Number of Args : "<<config_loc.size()<<endl;
    if(config_loc.size()==2)
        config_file = config_loc.at(1).toUtf8().constData(); 
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
        double z_max = 0.0,z_avg = 0.0;
        for(int i=0;i<pointcloud->points.size();i++)
        {
            auto pt = pointcloud->points[i];
            if(pt.x==0.0&&pt.y==0.0&&pt.z==0.0)
                continue;
            if(pt.z>z_max)
                z_max=pt.z;
            z_avg+=pt.z;
            temp_cloud->points.push_back(pt);
        }
        cout<<"Max-Z height: "<<z_max<<endl;
        cout<<"Average Z: "<<z_avg/temp_cloud->points.size()<<endl;
        clouds_.push_back(temp_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw_temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*temp_cloud,*cloud_bw_temp);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_bw_temp);
        constexpr double leaf = 0.007f;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.filter (*cloud_filtered);

        cloud_downsampled_.push_back(cloud_filtered);

        PointCloudT::Ptr pointcloud_output(new PointCloudT);
        cloud_outputs_.push_back(pointcloud_output);
        cout<<filename<<endl;
    }
    string ik_filename  = pt.get<std::string>("data.camera.transformations.inverse_kinematics");
    cout<<ik_filename<<endl;
    inverse_kinematics_ = readTransformations(ik_filename,true);
    cout<<"Transformations Read"<<endl;

    //Reading the scan cloud.
    string object_metric = pt.get<std::string>("data.scan.metric","m");
    cout<<"------------- "<<object_metric <<endl;
    for(const auto &cloud :pt.get_child("data.scan.clouds"))
    {
        string filename = cloud.second.data();
        readPointCloud(filename,cloud_,object_metric );
    }
    //Reflecting the cloud here. TODO: Need to remove this.
    Eigen::MatrixXd reflect_cloud= Eigen::MatrixXd::Identity(4,4);
    reflect_cloud(0,0) = 1;
    transformPointCloud<pcl::PointXYZRGB>(cloud_,reflect_cloud);

    cout<<"Scan read"<<endl; 
#if 0
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_,*cloud_bw);
    object_tree_.setInputCloud (cloud_bw);

    for(int i=0;i<clouds_.size();i++)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> object_tree; 
        object_tree.setInputCloud (cloud_bw);
        object_tree.setEpsilon(0.1);
        object_tree_vec_.push_back(object_tree);
        // pcl::KdTreeFLANN<pcl::PointXYZ> object_tree2; 
        // object_tree2.setInputCloud (cloud_bw);
        // object_tree_vec2_.push_back(object_tree2);
        // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        // tree->setInputCloud(cloud_bw);
        // tree->setEpsilon(0.1);
        // object_kdtree_vec_.push_back(tree);
    }
#endif

    MatrixXf M = MatrixXf::Zero(3, cloud_->points.size());
    for(int i=0;i<cloud_->points.size();i++)
    {
        auto pt = cloud_->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    kd_tree_nabo_ = NNSearchF::createKDTreeLinearHeap(M);

    cout<<object_tree_vec_.size()<<" trees are added."<<endl;

    double scale = 1.0;
    if(object_metric=="mm")
        scale = 1000.0;
    else if(object_metric=="cm")
        scale = 100.0;
    //Reading locations of the object.
    object_location_.reset(new PointCloudT);
    for(const auto &location :pt.get_child("data.scan.location"))
    {
        string loc = location.second.data();
        vector<string> coords;
        boost::split(coords, loc , boost::is_any_of(","));
        pcl::PointXYZRGB pt;
        pt.x = stof(coords[0])/scale;
        pt.y = stof(coords[1])/scale;
        pt.z = stof(coords[2])/scale;
        pt.r = pt.g = pt.b =0;
        object_location_->points.push_back(pt);
    }
    cout<<"Scan Location Read"<<endl;
    //Reading Default Values;
    auto transformation_initial_object= getTransVector(pt,"data.scan.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_object.size();i++)
    {
        transformation_[i]=transformation_initial_object[i];
        transformation_initial_[i] = transformation_initial_object[i];
    }

    auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_flange.size();i++)
    {
        flange_transformation_[i]=transformation_initial_flange[i];
        flange_transformation_initial_[i]=transformation_initial_flange[i];
    }
}

void PCLViewer::setupViz()
{
    // Set up the QVTK window and the pcl visualizer
    ui_->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
    viewer_->setBackgroundColor (0.2,0.2,0.2);
    viewer_->initCameraParameters ();
    // viewer_->addCoordinateSystem(1.0,Eigen::Affine3f::Identity(),"origin");
    // viewer_->resetCamera ();
    //Draw Grid
    drawGrid(viewer_);
    cout<<"Grid drawn"<<endl;
    addClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_);
    cout<<"Cloud added"<<endl;
    addObjectToSpace(cloud_,cloud_output_,transformation_,viewer_);
    cout<<"Object added"<<endl;
    viewer_->addPointCloud (object_location_, "locations");
    cout<<"Viewer Set"<<endl;
    viewer_->registerPointPickingCallback (&PCLViewer::pointPickingEventOccurred,*this);
    ui_->qvtkWidget->update ();
}

void PCLViewer::setupInterface()
{
    //Setting up the dropdown for clouds.
    vector<string> values;
    for(int i=0;i<clouds_.size();i++)
        values.push_back("cloud "+to_string(i+1));
    values.push_back("object");
    //if the location is provided.TODO
    values.push_back("object location");
    model_clouds_ = new QStandardItemModel;
    for (int i = 0; i < values.size(); i++)
    {
        QStandardItem *item = new QStandardItem();
        QString temp = QString::fromUtf8(values[i].c_str());
        item->setText(temp);
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        items_.push_back(item);
        model_clouds_->setItem(i, item);
        selected_clouds_.push_back(false);
    }
    updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_,selected_clouds_);
    updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,false);
    if(selected_clouds_[selected_clouds_.size()-1]==false)
    {
        PointCloudT::Ptr temp(new PointCloudT);
        viewer_->updatePointCloud (temp,"locations");
    }
    cout<<"Dropdown set"<<endl;
    ui_->listView->setModel(model_clouds_);
    //Setting up the dropdown for the axes.
    vector<string> values_axes={"clouds","object"};
    for(int i=0;i<clouds_.size();i++)
        values_axes.push_back("flange "+to_string(i+1));
    values_axes.push_back("origin");
    model_axes_ = new QStandardItemModel;
    for(int i=0;i<values_axes.size();i++)
    {
        QStandardItem *item = new QStandardItem();
        QString temp = QString::fromUtf8(values_axes[i].c_str());
        item->setText(temp);
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        items_.push_back(item);
        model_axes_->setItem(i, item);
        selected_axes_.push_back(false);
    }
    cout<<"Dropdown for axes done"<<endl;
    ui_->listView_2->setModel(model_axes_);

    QStringList translation_resolutions = {"0.1", "1", "10", "100"};
    ui_->comboBox->addItems(translation_resolutions);
    ui_->comboBox->setCurrentIndex(1);

    QStringList rotation_resolutions = {"0.1", "1", "2", "3","4","10"};
    ui_->comboBox_2->addItems(rotation_resolutions);
    ui_->comboBox_2->setCurrentIndex(2);

    QStringList algorithms = {"None", "4 point Method-Object", "4 point Method-Flange", "Optimization"};
    ui_->comboBox_3->addItems(algorithms);
    ui_->comboBox_3->setCurrentIndex(0);
}

void PCLViewer::addWidgets()
{
    // Connect button and the function
    // Connect (ui_->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));
    // Connect sliders and their functions
    connect (ui_->horizontalSlider_X_Object, SIGNAL (valueChanged (int)), this, SLOT (xSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Y_Object, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
    connect (ui_->horizontalSlider_Z_Object, SIGNAL (valueChanged (int)), this, SLOT (zSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dx_Object, SIGNAL (valueChanged (int)), this, SLOT (dxSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dy_Object, SIGNAL (valueChanged (int)), this, SLOT (dySliderValueChanged (int)));
    connect (ui_->horizontalSlider_dz_Object, SIGNAL (valueChanged (int)), this, SLOT (dzSliderValueChanged (int)));

    connect (ui_->horizontalSlider_X_Object, SIGNAL (sliderReleased ()), this, SLOT (objectSliderReleased ()));
    connect (ui_->horizontalSlider_Y_Object, SIGNAL (sliderReleased ()), this, SLOT (objectSliderReleased ()));
    connect (ui_->horizontalSlider_Z_Object, SIGNAL (sliderReleased ()), this, SLOT (objectSliderReleased ()));
    connect (ui_->horizontalSlider_dx_Object, SIGNAL (sliderReleased ()), this, SLOT (objectSliderReleased ()));
    connect (ui_->horizontalSlider_dy_Object, SIGNAL (sliderReleased ()), this, SLOT (objectSliderReleased ()));
    connect (ui_->horizontalSlider_dz_Object, SIGNAL (sliderReleased ()), this, SLOT (objectSliderReleased ()));

    connect (ui_->horizontalSlider_X_Camera, SIGNAL (valueChanged (int)), this, SLOT (xCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Y_Camera, SIGNAL (valueChanged (int)), this, SLOT (yCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Z_Camera, SIGNAL (valueChanged (int)), this, SLOT (zCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dx_Camera, SIGNAL (valueChanged (int)), this, SLOT (dxCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dy_Camera, SIGNAL (valueChanged (int)), this, SLOT (dyCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dz_Camera, SIGNAL (valueChanged (int)), this, SLOT (dzCameraSliderValueChanged (int)));

    connect (ui_->horizontalSlider_X_Camera, SIGNAL (sliderReleased ()), this, SLOT (cameraSliderReleased ()));
    connect (ui_->horizontalSlider_Y_Camera, SIGNAL (sliderReleased ()), this, SLOT (cameraSliderReleased ()));
    connect (ui_->horizontalSlider_Z_Camera, SIGNAL (sliderReleased ()), this, SLOT (cameraSliderReleased ()));
    connect (ui_->horizontalSlider_dx_Camera, SIGNAL (sliderReleased ()), this, SLOT (cameraSliderReleased ()));
    connect (ui_->horizontalSlider_dy_Camera, SIGNAL (sliderReleased ()), this, SLOT (cameraSliderReleased ()));
    connect (ui_->horizontalSlider_dz_Camera, SIGNAL (sliderReleased ()), this, SLOT (cameraSliderReleased ()));

    connect (ui_->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    connect (model_clouds_, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelChanged(QStandardItem*)));

    connect (model_axes_, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelAxesChanged(QStandardItem*)));

    connect (ui_->checkBox,  SIGNAL (clicked ()), this, SLOT (enableErrorCalculation()));

    connect (ui_->comboBox, SIGNAL (currentIndexChanged(int)), this, SLOT (transResolutionChanged (int)));
    connect (ui_->comboBox_2, SIGNAL (currentIndexChanged(int)), this, SLOT (rotResolutionChanged (int)));
    connect (ui_->comboBox_3, SIGNAL (currentIndexChanged(int)), this, SLOT (algorithmSelected(int)));
    connect (ui_->pushButton_2,  SIGNAL (clicked ()), this, SLOT (applyAlgorithm()));

    cout<<"QT components loaded"<<endl;

    updateTranslationControls(ui_->horizontalSlider_X_Object,ui_->label_10,ui_->label_33,transformation_[0]);
    updateTranslationControls(ui_->horizontalSlider_Y_Object,ui_->label_11,ui_->label_34,transformation_[1]);
    updateTranslationControls(ui_->horizontalSlider_Z_Object,ui_->label_12,ui_->label_35,transformation_[2]);
    updateRotationControls(ui_->horizontalSlider_dx_Object,ui_->label_13,ui_->label_36,(transformation_[5]));
    updateRotationControls(ui_->horizontalSlider_dy_Object,ui_->label_14,ui_->label_37,(transformation_[4]));
    updateRotationControls(ui_->horizontalSlider_dz_Object,ui_->label_21,ui_->label_38,(transformation_[3]));

    updateTranslationControls(ui_->horizontalSlider_X_Camera,ui_->label_22,ui_->label_45,flange_transformation_[0]);
    updateTranslationControls(ui_->horizontalSlider_Y_Camera,ui_->label_23,ui_->label_40,flange_transformation_[1]);
    updateTranslationControls(ui_->horizontalSlider_Z_Camera,ui_->label_24,ui_->label_41,flange_transformation_[2]);
    updateRotationControls(ui_->horizontalSlider_dx_Camera,ui_->label_25,ui_->label_42,(flange_transformation_[5]));
    updateRotationControls(ui_->horizontalSlider_dy_Camera,ui_->label_26,ui_->label_43,(flange_transformation_[4]));
    updateRotationControls(ui_->horizontalSlider_dz_Camera,ui_->label_27,ui_->label_44,(flange_transformation_[3]));

    // Generate html for the table.
    tb_ = ui_->textBrowser;
    string htmlString = "<htmt><body><style>table, th, td {border: 1px solid black;}</style><center>Error Metrics</center><table><tr><th>Cloud Id</th><th>Avg Error</th><th>Max Error</th></tr>";
    QString html = QString::fromUtf8(htmlString.c_str());
    tb_->setHtml(html);
    pSliderValueChanged (2);
    ui_->qvtkWidget->update ();
}



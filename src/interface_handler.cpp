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
// #include "nabo/nabo.h"

using namespace std;
using namespace Eigen;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;
// using namespace Nabo;

string getSplit(string name, string character,int id)
{
    vector<string> values;
    boost::split(values,name,boost::is_any_of(character));
    if(id<0)
        id = values.size()+id;
    return values[id];
}

int getFileId(string filename)
{
    string file = getSplit(filename,"/",-1);
    file = getSplit(file,".",0);
    int number = stoi(getSplit(file,"_",1));
    return number;
}

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
    int counter = 0;
    for (const auto &cloud : pt.get_child("data.camera.clouds"))
    {
        string filename = cloud.second.data();
        int cloud_id = getFileId(filename); 
        cout<<"Cloud Number: "<<cloud_id<<endl;
        mapping_[counter++] = cloud_id-1;
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
    string ik_filename  = pt.get<std::string>("data.camera.transformations.inverse_kinematics.location");
    string transformation_metric = pt.get<std::string>("data.camera.transformations.inverse_kinematics.metric","m");
    cout<<ik_filename<<endl;
    inverse_kinematics_ = readTransformations(ik_filename,true,transformation_metric);
    cout<<"Transformations Read"<<endl;

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

    //Reading Default Values;
    auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_flange.size();i++)
    {
        flange_transformation_[i]=transformation_initial_flange[i];
        flange_transformation_initial_[i]=transformation_initial_flange[i];
    }
    auto plane_equation = getPlaneEquation(pt,"data.plane");
    if(plane_equation.size())
    {
        use_plane_ = true;
        plane_ = plane_equation;
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
    cout<<"Viewer Set"<<endl;
    viewer_->registerPointPickingCallback (&PCLViewer::pointPickingEventOccurred,*this);
    cout<<"Callback Done"<<endl;
    if(use_plane_)
    {
        pcl::ModelCoefficients::Ptr plane_1 (new pcl::ModelCoefficients);
        plane_1->values.resize (4);
        plane_1->values[0] = plane_[0];
        plane_1->values[1] = plane_[1];
        plane_1->values[2] = plane_[2];
        plane_1->values[3] = plane_[3];
        viewer_->addPlane (*plane_1, "plane_1", 0);
    }
    ui_->qvtkWidget->update ();
    cout<<"QT Set"<<endl;
}

void PCLViewer::setupInterface()
{
    //Setting up the dropdown for clouds.
    cout<<"Setting up dropdown for clouds"<<endl;
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
        selected_clouds_.push_back(true);
    }
    updateClouds();
    cout<<"Updating Clouds"<<endl;
    updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,false);
    cout<<"Updating Object"<<endl;
    // if(selected_clouds_[selected_clouds_.size()-1]==false)
    // {
    //     PointCloudT::Ptr temp(new PointCloudT);
    //     viewer_->updatePointCloud (temp,"locations");
    // }
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

    QStringList algorithms = {"None", "4 point Method-Object", "4 point Method-Flange", "Optimization","Show Errors"};
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
    connect (ui_->horizontalSlider_p_2, SIGNAL (valueChanged (int)), this, SLOT (zThreshSliderValueChanged  (int)));

    connect (model_clouds_, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelChanged(QStandardItem*)));

    connect (model_axes_, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelAxesChanged(QStandardItem*)));

    connect (ui_->checkBox,  SIGNAL (clicked ()), this, SLOT (enableErrorCalculation()));

    connect (ui_->checkBox_2,  SIGNAL (clicked ()), this, SLOT (enablePointsClassifier()));

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

void PCLViewer::updateClouds(vector<PointCloudT::Ptr> clouds)
{
    if(clouds_.size()>inverse_kinematics_.size())
    {
        throw "Size Mismatch Error.\n";
    }
    vector<PointCloudT::Ptr> processed;
    cout<<"Entering Loop"<<endl;
    for(int i=0;i<clouds_.size();i++)
    {
        cout<<"Inside Loop"<<endl;
        PointCloudT::Ptr cloud;
        if(identify_good_points_==false)
        {
            cout<<"Here"<<endl;
            cloud = clouds_[i];
            cout<<"Here"<<endl;
        }
        else 
            cloud = cloud_classified_[i];
        if(clouds.size())
            cloud = clouds[i];
        cout<<"Assigned clouds"<<endl;
        PointCloudT::Ptr pro(new PointCloudT);
        for(auto pt:cloud->points)
        {
            if(pt.z<double(z_threshold_)/100.0)
                pro->points.push_back(pt);
        }
        processed.push_back(pro);
    }
    cout<<"Processed Clouds"<<endl;
    for(int i=0;i<clouds_.size();i++)
    {
        cloud_outputs_[i]->clear();
        Eigen::MatrixXd cam_T_flange = TransformationUtilities::vectorToTransformationMatrix(flange_transformation_);
        Eigen::MatrixXd transformation = inverse_kinematics_[mapping_[i]]*cam_T_flange;
        if(selected_clouds_[i])
        {
            if(identify_good_points_==false)
                TransformationUtilities::transformPointCloud<pcl::PointXYZRGB>(processed[i],cloud_outputs_[i],transformation);
            else
                TransformationUtilities::transformPointCloud<pcl::PointXYZRGB>(processed[i],cloud_outputs_[i],transformation);
        }
        viewer_->updatePointCloud (cloud_outputs_[i], "cloud"+to_string(i+1));
    }
}

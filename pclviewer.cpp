#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "helpers.hpp"
#include <pcl/io/ply_io.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>

using namespace std;
using namespace Eigen;
using namespace Helpers;


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
    // Reading the pointclouds, sensor scans and the transformations from files using XML config file.
    ifstream file;
    file.open("/home/rex/REX_WS/Catkin_WS/src/CameraCalibration/config/config.xml");
    using boost::property_tree::ptree;
    ptree pt;
    read_xml(file, pt);
    for (const auto &cloud : pt.get_child("data.camera.clouds"))
    {
        string filename = cloud.second.data();
        PointCloudT::Ptr pointcloud(new PointCloudT);
        if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *pointcloud) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file for base. \n");
        }
        clouds_.push_back(pointcloud);
        PointCloudT::Ptr pointcloud_output(new PointCloudT);
        cloud_outputs_.push_back(pointcloud_output);
        cout<<filename<<endl;
    }
    for(const auto &transformation : pt.get_child("data.camera.transformations"))
    {
        string ik_filename = transformation.second.data();
        inverse_kinematics_ = readTransformations(ik_filename);
    }
    for(const auto &cloud :pt.get_child("data.scan.clouds"))
    {
        string filename = cloud.second.data();
        pcl::PLYReader reader;
        reader.read(filename, *cloud_);
    }
    // Set up the QVTK window and the pcl visualizer
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui_->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
    viewer_->setBackgroundColor (255,255,255);
    viewer_->initCameraParameters ();
    viewer_->addCoordinateSystem(1.0);
    // viewer_->resetCamera ();
    //Draw Grid
    Helpers::drawGrid(viewer_);
    addClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_);
    addObjectToSpace(cloud_,cloud_output_,transformation_,viewer_);
    ui_->qvtkWidget->update ();
    //Setting up the dropdown for clouds.
    vector<string> values;
    for(int i=0;i<clouds_.size();i++)
        values.push_back("cloud "+to_string(i));
    values.push_back("object");
    QStandardItemModel *model = new QStandardItemModel;
    for (int i = 0; i < values.size(); i++)
    {
        QStandardItem *item = new QStandardItem();
        QString temp = QString::fromUtf8(values[i].c_str());
        item->setText(temp);
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        items_.push_back(item);
        model->setItem(i, item);
        selected_clouds_.push_back(false);
    }
    updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_,selected_clouds_);
    updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,false);
    ui_->listView->setModel(model);
    //Setting up the dropdown for the axes.
    vector<string> values_axes={"clouds","object"};
    for(int i=0;i<clouds_.size();i++)
        values_axes.push_back("flange "+to_string(i));
    QStandardItemModel *model_axes = new QStandardItemModel;
    for(int i=0;i<values_axes.size();i++)
    {
        QStandardItem *item = new QStandardItem();
        QString temp = QString::fromUtf8(values_axes[i].c_str());
        item->setText(temp);
        item->setCheckable(true);
        item->setCheckState(Qt::Unchecked);
        items_.push_back(item);
        model_axes->setItem(i, item);
        selected_axes_.push_back(false);
    }
    ui_->listView_2->setModel(model_axes);
    // Connect button and the function
    // Connect (ui_->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));
    // Connect sliders and their functions
    connect (ui_->horizontalSlider_X_Object, SIGNAL (valueChanged (int)), this, SLOT (xSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Y_Object, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
    connect (ui_->horizontalSlider_Z_Object, SIGNAL (valueChanged (int)), this, SLOT (zSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dx_Object, SIGNAL (valueChanged (int)), this, SLOT (dxSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dy_Object, SIGNAL (valueChanged (int)), this, SLOT (dySliderValueChanged (int)));
    connect (ui_->horizontalSlider_dz_Object, SIGNAL (valueChanged (int)), this, SLOT (dzSliderValueChanged (int)));

    connect (ui_->horizontalSlider_X_Object, SIGNAL (sliderReleased ()), this, SLOT (ObjectsliderReleased ()));
    connect (ui_->horizontalSlider_Y_Object, SIGNAL (sliderReleased ()), this, SLOT (ObjectsliderReleased ()));
    connect (ui_->horizontalSlider_Z_Object, SIGNAL (sliderReleased ()), this, SLOT (ObjectsliderReleased ()));
    connect (ui_->horizontalSlider_dx_Object, SIGNAL (sliderReleased ()), this, SLOT (ObjectsliderReleased ()));
    connect (ui_->horizontalSlider_dy_Object, SIGNAL (sliderReleased ()), this, SLOT (ObjectsliderReleased ()));
    connect (ui_->horizontalSlider_dz_Object, SIGNAL (sliderReleased ()), this, SLOT (ObjectsliderReleased ()));

    connect (ui_->horizontalSlider_X_Camera, SIGNAL (valueChanged (int)), this, SLOT (xCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Y_Camera, SIGNAL (valueChanged (int)), this, SLOT (yCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Z_Camera, SIGNAL (valueChanged (int)), this, SLOT (zCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dx_Camera, SIGNAL (valueChanged (int)), this, SLOT (dxCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dy_Camera, SIGNAL (valueChanged (int)), this, SLOT (dyCameraSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dz_Camera, SIGNAL (valueChanged (int)), this, SLOT (dzCameraSliderValueChanged (int)));

    connect (ui_->horizontalSlider_X_Camera, SIGNAL (sliderReleased ()), this, SLOT (CamerasliderReleased ()));
    connect (ui_->horizontalSlider_Y_Camera, SIGNAL (sliderReleased ()), this, SLOT (CamerasliderReleased ()));
    connect (ui_->horizontalSlider_Z_Camera, SIGNAL (sliderReleased ()), this, SLOT (CamerasliderReleased ()));
    connect (ui_->horizontalSlider_dx_Camera, SIGNAL (sliderReleased ()), this, SLOT (CamerasliderReleased ()));
    connect (ui_->horizontalSlider_dy_Camera, SIGNAL (sliderReleased ()), this, SLOT (CamerasliderReleased ()));
    connect (ui_->horizontalSlider_dz_Camera, SIGNAL (sliderReleased ()), this, SLOT (CamerasliderReleased ()));

    connect (ui_->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    connect (model, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelChanged(QStandardItem*)));

    connect (model_axes, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelAxesChanged(QStandardItem*)));

    // Generate html for the table.
    tb_ = ui_->textBrowser;
    string htmlString = "<htmt><body><style>table, th, td {border: 1px solid black;}</style><center>Error Metrics</center>"+getErrorMetrics(cloud_outputs_,cloud_)+"</body></html>";
    QString html = QString::fromUtf8(htmlString.c_str());
    tb_->setHtml(html);
    pSliderValueChanged (2);
    ui_->qvtkWidget->update ();
}

void addCoordinateAxes(Eigen::MatrixXd& transformation,pcl::visualization::PCLVisualizer::Ptr viewer,string id)
{
    if(viewer->contains(id))
        viewer->removeCoordinateSystem(id);
    viewer->addCoordinateSystem(0.25,getAffineMatrix(transformation),id);
}

void PCLViewer::updateAxes()
{
    if(selected_axes_[0])
    {
        for(int i=0;i<clouds_.size();i++)
        {
            Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation_);
            Eigen::MatrixXd transformation = inverse_kinematics_[i]*cam_T_flange;
            if(selected_clouds_[i])
                addCoordinateAxes(transformation,viewer_,"camera"+to_string(i));
            else
                if(viewer_->contains("camera"+to_string(i)))
                    viewer_->removeCoordinateSystem("camera"+to_string(i));
        }
    }
    else
    {
        for(int i=0;i<clouds_.size();i++)
        {
            if(viewer_->contains("camera"+to_string(i)))
                viewer_->removeCoordinateSystem("camera"+to_string(i));
        }
    }
    if(selected_axes_[1])
    {
        if(selected_clouds_[selected_clouds_.size()-1])
        {
            Eigen::MatrixXd part_T_base = vectorToTransformationMatrix(transformation_);
            addCoordinateAxes(part_T_base ,viewer_,"object");
        }
        else
        {
            if(viewer_->contains("object"))
                viewer_->removeCoordinateSystem("object");
        }
    }
    else
    {
            if(viewer_->contains("object"))
                viewer_->removeCoordinateSystem("object");
    }
    for(int i=2;i<selected_axes_.size();i++)
        if(selected_axes_[i])
            addCoordinateAxes(inverse_kinematics_[i-2],viewer_,"flange"+to_string(i-2));
        else
            if(viewer_->contains("flange"+to_string(i-2)))
                viewer_->removeCoordinateSystem("flange"+to_string(i-2));
}

void PCLViewer::modelAxesChanged (QStandardItem *item)
{
    cout<<"Here"<<endl;
    auto cs = item->checkState();
    auto id = item->index();
    cout<<cs<<endl;
    cout<<id.row()<<endl;
    if(cs)
        selected_axes_[id.row()]=true;
    else
        selected_axes_[id.row()]=false;
    //TODO: Update IK axis
    updateAxes();
    ui_->qvtkWidget->update ();
}

void PCLViewer::modelChanged (QStandardItem *item)
{
    cout<<"Here"<<endl;
    auto cs = item->checkState();
    auto id = item->index();
    if(cs)
        selected_clouds_[id.row()]=true;
    else
        selected_clouds_[id.row()]=false;
    if(id.row()<clouds_.size())
        updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_,selected_clouds_);
    else
        updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,selected_clouds_[id.row()]);
    updateAxes();
    ui_->qvtkWidget->update ();
}

    void
PCLViewer::ObjectsliderReleased ()
{
    // Set the new clouds
    updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_);
    updateAxes();
    ui_->qvtkWidget->update ();
    for(int i=0;i<transformation_.size();i++)
        cout<<transformation_[i]<<" ";
    cout<<endl;
    cout<<"Updated"<<endl;
}
    void
PCLViewer::CamerasliderReleased ()
{
    // Set the new clouds
    updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_,selected_clouds_);
    updateAxes();
    ui_->qvtkWidget->update ();
    for(int i=0;i<flange_transformation_.size();i++)
        cout<<flange_transformation_[i]<<" ";
    cout<<endl;
    cout<<"Updated"<<endl;
}

    void
PCLViewer::pSliderValueChanged (int value)
{
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "object");
    for(int i=0;i<clouds_.size();i++)
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud"+to_string(i));
    updateAxes();
    ui_->qvtkWidget->update ();
}

    void
PCLViewer::xSliderValueChanged (int value)
{
    transformation_[0]=value/100.0;
}

    void
PCLViewer::ySliderValueChanged (int value)
{
    transformation_[1]=value/100.0;
}

    void
PCLViewer::zSliderValueChanged (int value)
{
    transformation_[2]=value/100.0;
}
    void
PCLViewer::dxSliderValueChanged (int value)
{
    transformation_[5]=Helpers::degreeToRadian(value);
}
    void
PCLViewer::dySliderValueChanged (int value)
{
    transformation_[4]=Helpers::degreeToRadian(value);
}
    void
PCLViewer::dzSliderValueChanged (int value)
{
    transformation_[3]=Helpers::degreeToRadian(value);
}

    void
PCLViewer::xCameraSliderValueChanged (int value)
{
    flange_transformation_[0]=value/100.0;
    cout<<"Changed"<<endl;
}

    void
PCLViewer::yCameraSliderValueChanged (int value)
{
    flange_transformation_[1]=value/100.0;
}

    void
PCLViewer::zCameraSliderValueChanged (int value)
{
    flange_transformation_[2]=value/100.0;
}
    void
PCLViewer::dxCameraSliderValueChanged (int value)
{
    flange_transformation_[5]=Helpers::degreeToRadian(value);
}
    void
PCLViewer::dyCameraSliderValueChanged (int value)
{
    flange_transformation_[4]=Helpers::degreeToRadian(value);
}
    void
PCLViewer::dzCameraSliderValueChanged (int value)
{
    flange_transformation_[3]=Helpers::degreeToRadian(value);
}

PCLViewer::~PCLViewer ()
{
    delete ui_;
}

// for(auto x:inverse_kinematics_)
// {
//     for(int i=0;i<4;i++)
//     {
//         for(int j=0;j<4;j++)
//             cout<<x(i,j)<<" ";
//         cout<<endl;
//     }
//     cout<<endl;
// }
//
//
//     void
// PCLViewer::randomButtonPressed ()
// {
//     printf ("Random button was pressed\n");
//
//     // Set the new color
//     for (auto& point: *cloud_)
//     {
//         point.r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
//         point.g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
//         point.b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
//     }
//
//     viewer_->updatePointCloud (cloud_, "cloud");
//     ui_->qvtkWidget->update ();
// }

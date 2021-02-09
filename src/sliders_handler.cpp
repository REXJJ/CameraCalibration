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

    void
PCLViewer::objectSliderReleased ()
{
    // Set the new clouds
    updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_);
    updateAxes();
    if(calculate_error_)
        updateErrorTable();
    ui_->qvtkWidget->update ();
    cout<<"Transformation Values: ";
    for(int i=0;i<transformation_.size();i++)
        cout<<transformation_[i]<<" ";
    cout<<endl;
    cout<<"Updated"<<endl;
}
    void
PCLViewer::cameraSliderReleased ()
{
    // Set the new clouds
    updateClouds();
    updateAxes();
    if(calculate_error_)
        updateErrorTable();
    ui_->qvtkWidget->update ();
    cout<<"Transformation Values: ";
    for(int i=0;i<flange_transformation_.size();i++)
        cout<<flange_transformation_[i]<<" ";
    cout<<endl;
    cout<<"Updated"<<endl;
}

    void
PCLViewer::pSliderValueChanged (int value)
{
    ui_->label_30->setNum(value);
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "object");
    for(int i=0;i<clouds_.size();i++)
        viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud"+to_string(i+1));
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "locations");
    updateAxes();
    ui_->qvtkWidget->update ();
}

void 
PCLViewer::zThreshSliderValueChanged (int value)
{
    ui_->label_46->setNum(value);
    z_threshold_ = value;
    updateClouds();
}

    void
PCLViewer::xSliderValueChanged (int value)
{
    transformation_[0]=updateTranslationControls(ui_->horizontalSlider_X_Object,ui_->label_10,ui_->label_33,transformation_initial_[0]);
}

    void
PCLViewer::ySliderValueChanged (int value)
{
    transformation_[1]=updateTranslationControls(ui_->horizontalSlider_Y_Object,ui_->label_11,ui_->label_34,transformation_initial_[1]);
}

    void
PCLViewer::zSliderValueChanged (int value)
{
    transformation_[2]=updateTranslationControls(ui_->horizontalSlider_Z_Object,ui_->label_12,ui_->label_35,transformation_initial_[2]);
}
    void
PCLViewer::dxSliderValueChanged (int value)
{
    transformation_[5]=updateRotationControls(ui_->horizontalSlider_dx_Object,ui_->label_13,ui_->label_36,transformation_initial_[5]);
}
    void
PCLViewer::dySliderValueChanged (int value)
{
    transformation_[4]=updateRotationControls(ui_->horizontalSlider_dy_Object,ui_->label_14,ui_->label_37,transformation_initial_[4]);
}
    void
PCLViewer::dzSliderValueChanged (int value)
{
    transformation_[3]=updateRotationControls(ui_->horizontalSlider_dz_Object,ui_->label_21,ui_->label_38,transformation_initial_[3]);
}

    void
PCLViewer::xCameraSliderValueChanged (int value)
{
    flange_transformation_[0]=updateTranslationControls(ui_->horizontalSlider_X_Camera,ui_->label_22,ui_->label_45,flange_transformation_initial_[0]);
}

    void
PCLViewer::yCameraSliderValueChanged (int value)
{
    flange_transformation_[1]=updateTranslationControls(ui_->horizontalSlider_Y_Camera,ui_->label_23,ui_->label_40,flange_transformation_initial_[1]);
}

    void
PCLViewer::zCameraSliderValueChanged (int value)
{
    flange_transformation_[2]=updateTranslationControls(ui_->horizontalSlider_Z_Camera,ui_->label_24,ui_->label_41,flange_transformation_initial_[2]);
}
    void
PCLViewer::dxCameraSliderValueChanged (int value)
{
    flange_transformation_[5]=updateRotationControls(ui_->horizontalSlider_dx_Camera,ui_->label_25,ui_->label_42,flange_transformation_initial_[5]);
}
    void
PCLViewer::dyCameraSliderValueChanged (int value)
{
    flange_transformation_[4]=updateRotationControls(ui_->horizontalSlider_dy_Camera,ui_->label_26,ui_->label_43,flange_transformation_initial_[4]);
}
    void
PCLViewer::dzCameraSliderValueChanged (int value)
{
    flange_transformation_[3]=updateRotationControls(ui_->horizontalSlider_dz_Camera,ui_->label_27,ui_->label_44,flange_transformation_initial_[3]);
}

double PCLViewer::updateTranslationControls(QSlider *slider,QLabel *label,QLabel *label2,double initial)
{
    int value = slider->value();
    double display = initial*1000.0+(value)*translation_resolution_;
    label->setNum(display);
    label2->setNum(double(value*translation_resolution_));
    return display/1000.0;
}

double PCLViewer::updateRotationControls(QSlider *slider,QLabel *label,QLabel *label2,double initial)
{
    int value = slider->value();
    double display = radTodeg(initial)+(value)*rotation_resolution_;
    label->setNum(display);
    label2->setNum(double(value*rotation_resolution_));
    return degreeToRadian(display);
}

void PCLViewer::transResolutionChanged(int value)
{
    vector<double> resolutions = {0.1,1.0,10.0,100.0};
    for(int i=0;i<3;i++)
    {
        transformation_initial_[i]=transformation_[i];
        flange_transformation_initial_[i]=flange_transformation_[i];
    }
    ui_->horizontalSlider_X_Object->setValue(0);
    ui_->horizontalSlider_Y_Object->setValue(0);
    ui_->horizontalSlider_Z_Object->setValue(0);
    ui_->horizontalSlider_X_Camera->setValue(0);
    ui_->horizontalSlider_Y_Camera->setValue(0);
    ui_->horizontalSlider_Z_Camera->setValue(0);
    translation_resolution_ = resolutions[value];
}

void PCLViewer::rotResolutionChanged(int value)
{
    vector<double> resolutions = {0.1,1.0,2.0,3.0,4.0,10.0};
    for(int i=3;i<6;i++)
    {
        transformation_initial_[i]=transformation_[i];
        flange_transformation_initial_[i]=flange_transformation_[i];
    }
    ui_->horizontalSlider_dx_Object->setValue(0);
    ui_->horizontalSlider_dy_Object->setValue(0);
    ui_->horizontalSlider_dz_Object->setValue(0);
    ui_->horizontalSlider_dx_Camera->setValue(0);
    ui_->horizontalSlider_dy_Camera->setValue(0);
    ui_->horizontalSlider_dz_Camera->setValue(0);
    rotation_resolution_ = resolutions[value];
}

void PCLViewer::updateSliders()
{
    ui_->horizontalSlider_X_Object->setValue(0);
    ui_->horizontalSlider_Y_Object->setValue(0);
    ui_->horizontalSlider_Z_Object->setValue(0);
    ui_->horizontalSlider_dx_Object->setValue(0);
    ui_->horizontalSlider_dy_Object->setValue(0);
    ui_->horizontalSlider_dz_Object->setValue(0);
    updateTranslationControls(ui_->horizontalSlider_X_Object,ui_->label_10,ui_->label_33,transformation_[0]);
    updateTranslationControls(ui_->horizontalSlider_Y_Object,ui_->label_11,ui_->label_34,transformation_[1]);
    updateTranslationControls(ui_->horizontalSlider_Z_Object,ui_->label_12,ui_->label_35,transformation_[2]);
    updateRotationControls(ui_->horizontalSlider_dx_Object,ui_->label_13,ui_->label_36,(transformation_[5]));
    updateRotationControls(ui_->horizontalSlider_dy_Object,ui_->label_14,ui_->label_37,(transformation_[4]));
    updateRotationControls(ui_->horizontalSlider_dz_Object,ui_->label_21,ui_->label_38,(transformation_[3]));
}

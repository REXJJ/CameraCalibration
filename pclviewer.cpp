#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "helpers.hpp"
#include <pcl/io/ply_io.h>


using namespace std;

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui_ (new Ui::PCLViewer)
{
    ui_->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Setup the cloud pointer
    cloud_.reset (new PointCloudT);
    cloud_output_.reset (new PointCloudT);

    // Reading the pointclouds, sensor scans and the transformations.
    pcl::PLYReader reader;
    reader.read("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/empty_box_inside/1.ply", *cloud_);
    transformation_ = std::vector<double>(6,0);

    // Set up the QVTK window and the pcl visualizer
    viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui_->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
    viewer_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
    viewer_->setBackgroundColor (255,255,255);
    viewer_->initCameraParameters ();
    viewer_->addCoordinateSystem(1.0);
    viewer_->addPointCloud (cloud_, "cloud");
    // viewer_->resetCamera ();
    //Draw Grid
    Helpers::drawGrid(viewer_);

    ui_->qvtkWidget->update ();

    // Connect button and the function
    // Connect (ui_->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

    // Connect sliders and their functions
    connect (ui_->horizontalSlider_X_Object, SIGNAL (valueChanged (int)), this, SLOT (xSliderValueChanged (int)));
    connect (ui_->horizontalSlider_Y_Object, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
    connect (ui_->horizontalSlider_Z_Object, SIGNAL (valueChanged (int)), this, SLOT (zSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dx_Object, SIGNAL (valueChanged (int)), this, SLOT (dxSliderValueChanged (int)));
    connect (ui_->horizontalSlider_dy_Object, SIGNAL (valueChanged (int)), this, SLOT (dySliderValueChanged (int)));
    connect (ui_->horizontalSlider_dz_Object, SIGNAL (valueChanged (int)), this, SLOT (dzSliderValueChanged (int)));

    connect (ui_->horizontalSlider_X_Object, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui_->horizontalSlider_Y_Object, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui_->horizontalSlider_Z_Object, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui_->horizontalSlider_dx_Object, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui_->horizontalSlider_dy_Object, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui_->horizontalSlider_dz_Object, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

    connect (ui_->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    // Generate html for the table.
    tb_ = ui_->textBrowser;
    QString htmlString = "<htmt><body>Hello, World</body></html>";
    tb_->setHtml(htmlString);

    pSliderValueChanged (2);
    ui_->qvtkWidget->update ();
}

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

    void
PCLViewer::RGBsliderReleased ()
{
    // Set the new clouds
    //
    cloud_output_->clear();
    Eigen::MatrixXd part_T_base = Helpers::vectorToTransformationMatrix(transformation_);
    Helpers::transformPointCloud<pcl::PointXYZRGB>(cloud_,cloud_output_,part_T_base);
    viewer_->updatePointCloud (cloud_output_, "cloud");
    ui_->qvtkWidget->update ();
    for(int i=0;i<transformation_.size();i++)
        cout<<transformation_[i]<<" ";
    cout<<endl;
    cout<<"Updated"<<endl;
}

    void
PCLViewer::pSliderValueChanged (int value)
{
    viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
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

PCLViewer::~PCLViewer ()
{
    delete ui_;
}

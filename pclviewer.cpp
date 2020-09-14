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

void addClouds(vector<PointCloudT::Ptr>& clouds,vector<PointCloudT::Ptr>& cloud_outputs,vector<MatrixXd>& iks,vector<double> & flange_transformation,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    if(clouds.size()>iks.size())
    {
        throw "Size Mismatch Error.\n";
    }
    for(int i=0;i<clouds.size();i++)
    {
        cloud_outputs[i]->clear();
        Eigen::MatrixXd cam_T_flange = Helpers::vectorToTransformationMatrix(flange_transformation);
        Eigen::MatrixXd transformation = iks[i]*cam_T_flange;
        Helpers::transformPointCloud<pcl::PointXYZRGB>(clouds[i],cloud_outputs[i],transformation);
        viewer->addPointCloud (cloud_outputs[i], "cloud"+to_string(i));
    }
}

void updateClouds(vector<PointCloudT::Ptr>& clouds,vector<PointCloudT::Ptr>& cloud_outputs,vector<MatrixXd>& iks,vector<double> & flange_transformation,pcl::visualization::PCLVisualizer::Ptr viewer)
{
    if(clouds.size()>iks.size())
    {
        throw "Size Mismatch Error.\n";
    }
    for(int i=0;i<clouds.size();i++)
    {
        cloud_outputs[i]->clear();
        Eigen::MatrixXd cam_T_flange = Helpers::vectorToTransformationMatrix(flange_transformation);
        Eigen::MatrixXd transformation = iks[i]*cam_T_flange;
        Helpers::transformPointCloud<pcl::PointXYZRGB>(clouds[i],cloud_outputs[i],transformation);
        viewer->updatePointCloud (cloud_outputs[i], "cloud"+to_string(i));
    }
}

vector<MatrixXd> readTransformations(string filename)
{
    vector<MatrixXd> transformations;
	ifstream file(filename);
	MatrixXd mat=MatrixXd::Zero(4,4);
	string line;
	int count=0;
	while(true)
	{
        for(int i=0;i<4;i++)
        {
            getline(file,line);
            if(line.size()==0)
                goto end;
            vector<string> v;
            split(v,line,boost::is_any_of(","));
            for(int j=0;j<v.size();j++)
                mat(i,j)=stof(v[j]);
        }
        transformations.push_back(mat);
	}
end:
    return transformations;
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
    transformation_ = std::vector<double>(6,0);
    flange_transformation_ = std::vector<double>(6,0);

    ifstream file;
    file.open("/home/rex/REX_WS/Catkin_WS/src/CameraCalibration/config/config.xml");
    using boost::property_tree::ptree;
    ptree pt;
    read_xml(file, pt);
    // traverse pt
    for (const auto &cloud : pt.get_child("data.camera.clouds"))
    {
        string filename = cloud.second.data();
        PointCloudT::Ptr pointcloud(new PointCloudT);
        // reader.read(filename,*pointcloud);
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
        reader.read(filename, *cloud_);
    }

    //
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

    addClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_);

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

    // Generate html for the table.
    tb_ = ui_->textBrowser;
    string htmlString = "<htmt><body><style>table, th, td {border: 1px solid black;}</style><center>Error Metrics</center>"+getErrorMetrics(cloud_outputs_,cloud_)+"</body></html>";
    QString html = QString::fromUtf8(htmlString.c_str());
    tb_->setHtml(html);
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
PCLViewer::ObjectsliderReleased ()
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
PCLViewer::CamerasliderReleased ()
{
    // Set the new clouds
    updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_);
    ui_->qvtkWidget->update ();
    for(int i=0;i<flange_transformation_.size();i++)
        cout<<flange_transformation_[i]<<" ";
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

    void
PCLViewer::xCameraSliderValueChanged (int value)
{
    flange_transformation_[0]=value/100.0;
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

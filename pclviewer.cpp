#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <pcl/io/ply_io.h>

using namespace std;

namespace Helpers
{
    inline Eigen::MatrixXd apply_transformation(Eigen::MatrixXd data, Eigen::Matrix4d T_mat)
    {
        Eigen::MatrixXd data_with_fourth_row(data.cols()+1,data.rows());
        Eigen::VectorXd ones_vec = Eigen::VectorXd::Constant(data.rows(),1);
        data_with_fourth_row.block(0,0,data.cols(),data.rows()) = data.transpose();
        data_with_fourth_row.block(data.cols(),0,1,data.rows()) = ones_vec.transpose();
        Eigen::MatrixXd transformed_data = T_mat*data_with_fourth_row;
        Eigen::MatrixXd transformed_data_mat(transformed_data.rows()-1,transformed_data.cols());
        transformed_data_mat = transformed_data.block(0,0,transformed_data.rows()-1,transformed_data.cols());
        return transformed_data_mat.transpose();
    }

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
    std::string validate_seq(std::string seq)
	{
		if(seq =="")
			seq = "ZYX";	
		bool invalid_flag = false;
		if(seq.size()!=3)
		{
			invalid_flag = true;
		}
		for (int i =0;i<3;++i)
			if(seq[i]!='X' && seq[i]!='Y' && seq[i]!='Z' && seq[i]!='x' && seq[i]!='y' && seq[i]!='z')
			{
				invalid_flag = true; 
				break;
			}
		if(invalid_flag)
		{
			std::cerr << "ERROR: Invalid Rotations Sequence: " << seq << std::endl;
			std::terminate();		
		}
		return seq;
	}

    //Default : ZYX
    Eigen::Matrix3d eul2rot(Eigen::MatrixXd eul_angles, std::string seq="")
    {
        seq = validate_seq(seq);
        Eigen::Matrix3d rot_mat = Eigen::Matrix3d::Identity();
        for (int i=0; i<3; ++i)
        {
            if(seq[i]=='X' || seq[i]=='x')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitX());
            else if(seq[i]=='Y' || seq[i]=='y')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitY());			
            else if(seq[i]=='Z' || seq[i]=='z')
                rot_mat = rot_mat * Eigen::AngleAxisd(eul_angles(0,i), Eigen::Vector3d::UnitZ());					
        }
        return rot_mat; 
    }

    Eigen::MatrixXd vectorToTransformationMatrix(vector<double>& a_T_b_static)
    {
        Eigen::MatrixXd a_T_b = Eigen::MatrixXd::Identity(4,4);
        a_T_b(0,3) = a_T_b_static[0];        
        a_T_b(1,3) = a_T_b_static[1];
        a_T_b(2,3) = a_T_b_static[2];
        Eigen::MatrixXd eul_ang(1,3);
        eul_ang<<a_T_b_static[3],a_T_b_static[4],a_T_b_static[5];
        a_T_b.block(0,0,3,3) = eul2rot(eul_ang);
        return a_T_b;
    }

    constexpr double degreeToRadian(int angle)
    {
        return angle*3.141592/180;
    }
    
};

PCLViewer::PCLViewer (QWidget *parent) :
    QMainWindow (parent),
    ui (new Ui::PCLViewer)
{
    ui->setupUi (this);
    this->setWindowTitle ("PCL viewer");

    // Setup the cloud pointer
    cloud.reset (new PointCloudT);
    cloud_output.reset (new PointCloudT);
    // The number of points in the cloud
    // cloud->points.resize (200);
    pcl::PLYReader Reader;
    Reader.read("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/data/empty_box_inside/1.ply", *cloud);


    // The default color
    red   = 128;
    green = 128;
    blue  = 128;

    // Fill the cloud with some points
    // for (auto& point: *cloud)
    // {
    //     point.x = 1024 * rand () / (RAND_MAX + 1.0f);
    //     point.y = 1024 * rand () / (RAND_MAX + 1.0f);
    //     point.z = 1024 * rand () / (RAND_MAX + 1.0f);
    //
    //     point.r = red;
    //     point.g = green;
    //     point.b = blue;
    // }

    // Set up the QVTK window
    viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
    ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
    viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
    viewer->setBackgroundColor (255,255,255);
    // pcl::ModelCoefficients plane_coeff;
    // plane_coeff.values.resize (4);    // We need 4 values
    // plane_coeff.values[0] = 0;
    // plane_coeff.values[1] = 0;
    // plane_coeff.values[2] = 1;
    // plane_coeff.values[3] = 0;
    // viewer->addPlane (plane_coeff);
    ui->qvtkWidget->update ();

    // Connect "random" button and the function
    connect (ui->pushButton_random,  SIGNAL (clicked ()), this, SLOT (randomButtonPressed ()));

    // Connect R,G,B sliders and their functions
    connect (ui->horizontalSlider_R, SIGNAL (valueChanged (int)), this, SLOT (xSliderValueChanged (int)));
    connect (ui->horizontalSlider_G, SIGNAL (valueChanged (int)), this, SLOT (ySliderValueChanged (int)));
    connect (ui->horizontalSlider_B, SIGNAL (valueChanged (int)), this, SLOT (zSliderValueChanged (int)));
    connect (ui->horizontalSlider_dx, SIGNAL (valueChanged (int)), this, SLOT (dxSliderValueChanged (int)));
    connect (ui->horizontalSlider_dy, SIGNAL (valueChanged (int)), this, SLOT (dySliderValueChanged (int)));
    connect (ui->horizontalSlider_dz, SIGNAL (valueChanged (int)), this, SLOT (dzSliderValueChanged (int)));

    connect (ui->horizontalSlider_R, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_G, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_B, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_dx, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_dy, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));
    connect (ui->horizontalSlider_dz, SIGNAL (sliderReleased ()), this, SLOT (RGBsliderReleased ()));

    // Connect point size slider
    connect (ui->horizontalSlider_p, SIGNAL (valueChanged (int)), this, SLOT (pSliderValueChanged (int)));

    viewer->initCameraParameters ();
    viewer->addCoordinateSystem(1.0);
    viewer->addPointCloud (cloud, "cloud");
    pSliderValueChanged (2);
    // viewer->resetCamera ();
    ui->qvtkWidget->update ();
    transformation = std::vector<double>(6,0);
}

    void
PCLViewer::randomButtonPressed ()
{
    printf ("Random button was pressed\n");

    // Set the new color
    for (auto& point: *cloud)
    {
        point.r = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        point.g = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
        point.b = 255 *(1024 * rand () / (RAND_MAX + 1.0f));
    }

    viewer->updatePointCloud (cloud, "cloud");
    ui->qvtkWidget->update ();
}

    void
PCLViewer::RGBsliderReleased ()
{
    // Set the new clouds
    //
    cloud_output->clear();
    Eigen::MatrixXd part_T_base = Helpers::vectorToTransformationMatrix(transformation);
    Helpers::transformPointCloud<pcl::PointXYZRGB>(cloud,cloud_output,part_T_base);
    viewer->updatePointCloud (cloud_output, "cloud");
    ui->qvtkWidget->update ();
    for(int i=0;i<transformation.size();i++)
        cout<<transformation[i]<<" ";
    cout<<endl;
    cout<<"Updated"<<endl;
}

    void
PCLViewer::pSliderValueChanged (int value)
{
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud");
    ui->qvtkWidget->update ();
}

    void
PCLViewer::xSliderValueChanged (int value)
{
    transformation[0]=value/100.0;
}

    void
PCLViewer::ySliderValueChanged (int value)
{
    transformation[1]=value/100.0;
}

    void
PCLViewer::zSliderValueChanged (int value)
{
    transformation[2]=value/100.0;
}
    void
PCLViewer::dxSliderValueChanged (int value)
{
    transformation[5]=Helpers::degreeToRadian(value);
}
    void
PCLViewer::dySliderValueChanged (int value)
{
    transformation[4]=Helpers::degreeToRadian(value);
}
    void
PCLViewer::dzSliderValueChanged (int value)
{
    transformation[3]=Helpers::degreeToRadian(value);
}




PCLViewer::~PCLViewer ()
{
    delete ui;
}

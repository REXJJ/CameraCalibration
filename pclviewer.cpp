#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "helpers.hpp"
#include <pcl/io/ply_io.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <omp.h>

using namespace std;
using namespace Eigen;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;

void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
	std::cout << "[INOF] Point picking event occurred." << std::endl;

	float x, y, z;
	if (event.getPointIndex () == -1)
	{
		return;
	}
	event.getPoint(x, y, z);
	std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
}

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
	calculate_error_= false;
	viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));

    getInputs();
    setupViz();
    setupInterface();
    addWidgets();
}

void PCLViewer::enableErrorCalculation()
{
	calculate_error_=!calculate_error_;
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
				addCoordinateAxes(transformation,viewer_,"camera"+to_string(i+1));
			else
				if(viewer_->contains("camera"+to_string(i+1)))
					viewer_->removeCoordinateSystem("camera"+to_string(i+1));
		}
	}
	else
	{
		for(int i=0;i<clouds_.size();i++)
		{
			if(viewer_->contains("camera"+to_string(i+1)))
				viewer_->removeCoordinateSystem("camera"+to_string(i+1));
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
			addCoordinateAxes(inverse_kinematics_[i-2],viewer_,"flange"+to_string(i-1));
		else
			if(viewer_->contains("flange"+to_string(i-1)))
				viewer_->removeCoordinateSystem("flange"+to_string(i-1));
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
	auto cs = item->checkState();
	auto id = item->index();
	if(cs)
		selected_clouds_[id.row()]=true;
	else
		selected_clouds_[id.row()]=false;

	if(id.row()<clouds_.size())
		updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_,selected_clouds_);
	else if(id.row()==clouds_.size())
		updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,selected_clouds_[id.row()]);
    else
    {
        if(selected_clouds_[selected_clouds_.size()-1]==false)
        {
            PointCloudT::Ptr temp(new PointCloudT);
            viewer_->updatePointCloud (temp,"locations");
        }
        else
        {
            viewer_->updatePointCloud(object_location_,"locations");
        }
    }
    updateAxes();
	ui_->qvtkWidget->update ();
}

void PCLViewer::updateErrorTable()
{
	int K = 1;
	string htmlString = "<!DOCTYPE html><htmt><body><style>table, th, td { border: 1px solid black;border-collapse: collapse;}</style><center>Error Metrics</center><table><tr><th>Cloud Id</th><th>Avg Error</th><th>Max Error</th></tr>";
    vector<double> error_avg(clouds_.size(),1.0/0.0);
    vector<double> error_max(clouds_.size(),1.0/0.0);
    //TODO: Try parfor
#pragma omp parallel
#pragma omp for
    for(int j=0;j<clouds_.size();j++)
	{
		if(selected_clouds_[j]==false)
		{
			continue;
		}
		float average = 0.0;
		float maximum = -1e9;
		int counter = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation_);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation_);
        Eigen::MatrixXd transformation = inverse_kinematics_[j]*cam_T_flange;
        transformPointCloud<pcl::PointXYZ>(cloud_downsampled_[j],temp,transformation );
		world_T_object = world_T_object.inverse();
		transformPointCloud<pcl::PointXYZ>(temp,world_T_object);
		for(int i=0;i<temp->points.size();i+=1)
		{
			counter++;
			pcl::PointXYZ searchPoint = temp->points[i];
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
#if 0
			if ( object_kdtree_vec_[j]->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				float distance = sqrt(pointNKNSquaredDistance[0]);
				if(distance>maximum)
					maximum = distance;
				average+=distance;
			}
#else
			if ( object_tree_vec_[j].nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				float distance = sqrt(pointNKNSquaredDistance[0]);
				if(distance>maximum)
					maximum = distance;
				average+=distance;
			}
#endif
		}
        cout<<"Counted "<<counter<<" points."<<endl;
		average=average/float(counter);
        error_avg[j]=average;
        error_max[j]=maximum;
	}
    for(int j=0;j<error_avg.size();j++)
		htmlString +="<tr><td>"+to_string(j+1)+"</td><td>"+to_string(error_avg[j])+"</td><td>"+to_string(error_max[j])+"</td></tr>";
	htmlString+="</table></body></html>";
	QString html = QString::fromUtf8(htmlString.c_str());
	tb_->setHtml(html);
    cout<<"Error Table Updated."<<endl;
	// ui_->qvtkWidget->update ();
}

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
	updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_,selected_clouds_);
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
	viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "object");
	for(int i=0;i<clouds_.size();i++)
		viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "cloud"+to_string(i+1));
	viewer_->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, value, "locations");
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
	transformation_[5]=degreeToRadian(value);
}
	void
PCLViewer::dySliderValueChanged (int value)
{
	transformation_[4]=degreeToRadian(value);
}
	void
PCLViewer::dzSliderValueChanged (int value)
{
	transformation_[3]=degreeToRadian(value);
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
	flange_transformation_[5]=degreeToRadian(value);
}
	void
PCLViewer::dyCameraSliderValueChanged (int value)
{
	flange_transformation_[4]=degreeToRadian(value);
}
	void
PCLViewer::dzCameraSliderValueChanged (int value)
{
	flange_transformation_[3]=degreeToRadian(value);
}

PCLViewer::~PCLViewer ()
{
	delete ui_;
}

void PCLViewer::getInputs()
{
	// Reading the pointclouds, sensor scans and the transformations from files using XML config file.
	ifstream file;
	file.open("/home/rex/REX_WS/Catkin_WS/src/CameraCalibration/config/config.xml");
	// file.open("/home/rex/REX_WS/Test_WS/POINT_CLOUD_STITCHING/CameraCalibration/config/config.xml");
	using boost::property_tree::ptree;
	ptree pt;
	read_xml(file, pt);
	for (const auto &cloud : pt.get_child("data.camera.clouds"))
	{
		string filename = cloud.second.data();
		PointCloudT::Ptr pointcloud(new PointCloudT);
        readPointCloud(filename,pointcloud);

		PointCloudT::Ptr temp_cloud(new PointCloudT);
		for(int i=0;i<pointcloud->points.size();i++)
		{
			auto pt = pointcloud->points[i];
			if(pt.x==0.0&&pt.y==0.0&&pt.z==0.0)
				continue;
			temp_cloud->points.push_back(pt);
		}
		clouds_.push_back(temp_cloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw_temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*temp_cloud,*cloud_bw_temp);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_bw_temp);
        constexpr double leaf = 0.03f;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.filter (*cloud_filtered);

        cloud_downsampled_.push_back(cloud_filtered);

		PointCloudT::Ptr pointcloud_output(new PointCloudT);
		cloud_outputs_.push_back(pointcloud_output);
		cout<<filename<<endl;
	}
	for(const auto &transformation : pt.get_child("data.camera.transformations"))
	{
		string ik_filename = transformation.second.data();
		cout<<ik_filename<<endl;
		inverse_kinematics_ = readTransformations(ik_filename,true);
	}
	cout<<"Transformations Read"<<endl;
	for(const auto &cloud :pt.get_child("data.scan.clouds"))
	{
            string filename = cloud.second.data();
            readPointCloud(filename,cloud_);
	}
    //Reflecting the cloud here. TODO: Need to remove this.
	Eigen::MatrixXd reflect_cloud= Eigen::MatrixXd::Identity(4,4);
	reflect_cloud(0,0) = -1;
	transformPointCloud<pcl::PointXYZRGB>(cloud_,reflect_cloud);

	cout<<"Scan read"<<endl; 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_,*cloud_bw);
	object_tree_.setInputCloud (cloud_bw);

    for(int i=0;i<clouds_.size();i++)
    {
        pcl::KdTreeFLANN<pcl::PointXYZ> object_tree; 
        object_tree.setInputCloud (cloud_bw);
        object_tree_vec_.push_back(object_tree);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_bw);
        tree->setEpsilon(0.0001);
        object_kdtree_vec_.push_back(tree);
    }

    cout<<object_tree_vec_.size()<<" trees are added."<<endl;

	//Reading locations of the object.
	object_location_.reset(new PointCloudT);
	for(const auto &location :pt.get_child("data.scan.location"))
	{
		string loc = location.second.data();
		vector<string> coords;
		boost::split(coords, loc , boost::is_any_of(","));
		pcl::PointXYZRGB pt;
		pt.x = stof(coords[0])/1000.0;
		pt.y = stof(coords[1])/1000.0;
		pt.z = stof(coords[2])/1000.0;
		pt.r = pt.g = pt.b =0;
		object_location_->points.push_back(pt);
	}
	cout<<"Scan Location Read"<<endl;
}

void PCLViewer::setupViz()
{
	// Set up the QVTK window and the pcl visualizer
	ui_->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
	viewer_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
	viewer_->setBackgroundColor (0.2,0.2,0.2);
	viewer_->initCameraParameters ();
	viewer_->addCoordinateSystem(1.0);
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
	viewer_->registerPointPickingCallback (pointPickingEventOccurred);
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
	cout<<"QT components loaded"<<endl;

	// Generate html for the table.
	tb_ = ui_->textBrowser;
	string htmlString = "<htmt><body><style>table, th, td {border: 1px solid black;}</style><center>Error Metrics</center><table><tr><th>Cloud Id</th><th>Avg Error</th><th>Max Error</th></tr>";
	QString html = QString::fromUtf8(htmlString.c_str());
	tb_->setHtml(html);
	pSliderValueChanged (2);
	ui_->qvtkWidget->update ();
}

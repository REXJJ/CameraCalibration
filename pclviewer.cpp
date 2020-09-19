#include "pclviewer.h"
#include "ui_pclviewer.h"
#include "helpers.hpp"
#include <pcl/io/ply_io.h>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>

using namespace std;
using namespace Eigen;
using namespace Helpers;

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
#if 1
		pcl::PLYReader reader;
		reader.read(filename, *pointcloud);
#else
		if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (filename, *pointcloud) == -1) //* load the file
		{
			PCL_ERROR ("Couldn't read file for base. \n");
		}
#endif
		PointCloudT::Ptr temp_cloud(new PointCloudT);
		for(int i=0;i<pointcloud->points.size();i++)
		{
			auto pt = pointcloud->points[i];
			if(pt.x==0.0&&pt.y==0.0&&pt.z==0.0)
				continue;
			temp_cloud->points.push_back(pt);
		}
		clouds_.push_back(temp_cloud);
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
		ifstream file(filename);
		string metrics;
		getline(file,metrics);
		double z_min=1e9,z_max=-1e9;
		while(getline(file,metrics)&&metrics.size())
		{
			vector<string> result; 
			boost::split(result,metrics, boost::is_any_of(" "));
			string temp = result[1];
			vector<string> coords;
			boost::split(coords, temp, boost::is_any_of(","));
			pcl::PointXYZRGB pt;
			pt.x = stof(coords[0])/1000.0;
			pt.y = stof(coords[1])/1000.0;
			pt.z = stof(coords[2])/1000.0;
			pt.r = 0; 
			pt.g = 0;
			pt.b = 0;
			if(pt.z<z_min)
				z_min = pt.z;
			if(pt.z>z_max)
				z_max=pt.z;
			cloud_->points.push_back(pt);
		}
		//Color adjustment based on depth.
		for(auto &pt:cloud_->points)
		{
			double scale = (pt.z-z_min)/(z_max-z_min);
			pt.r = pt.g = pt.b = int(255.0*scale);
		}
		// pcl::PLYReader reader;
		// reader.read(filename, *cloud_);
		// TODO: Generic pointcloud reader.
	}
	Eigen::MatrixXd temp_t = Eigen::MatrixXd::Identity(4,4);
	temp_t(0,0) = -1;
	transformPointCloud<pcl::PointXYZRGB>(cloud_,temp_t);
	cout<<"Scan read"<<endl; 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud_,*cloud_bw);
	object_tree_.setInputCloud (cloud_bw);

	//Reading locations of the object.
	PointCloudT::Ptr object_location(new PointCloudT);
	for(const auto &location :pt.get_child("data.scan.location"))
	{
		string loc = location.second.data();
		vector<string> coords;
		boost::split(coords, loc , boost::is_any_of(","));
		pcl::PointXYZRGB pt;
		pt.x = stof(coords[0])/1000.0;
		pt.y = stof(coords[1])/1000.0;
		pt.z = stof(coords[2])/1000.0;
		cout<<pt.x<<" "<<pt.y<<" "<<pt.z<<endl;
		pt.r = pt.g = pt.b =0;
		object_location->points.push_back(pt);
	}
	cout<<"Scan Location Read"<<endl;
	// Set up the QVTK window and the pcl visualizer
	viewer_.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
	ui_->qvtkWidget->SetRenderWindow (viewer_->getRenderWindow ());
	viewer_->setupInteractor (ui_->qvtkWidget->GetInteractor (), ui_->qvtkWidget->GetRenderWindow ());
	viewer_->setBackgroundColor (0.2,0.2,0.2);
	viewer_->initCameraParameters ();
	viewer_->addCoordinateSystem(1.0);
	// pcl::visualization::PCLVisualizer viewer = *viewer_;
	// viewer_->registerPointPickingCallback (pointPickingEventOccurred, (void*)&viewer);
	// viewer_->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 100, 100, 100, "mesh_id");
	// viewer_->resetCamera ();
	//Draw Grid
	Helpers::drawGrid(viewer_);
	cout<<"Grid drawn"<<endl;
	addClouds(clouds_,cloud_outputs_,inverse_kinematics_,flange_transformation_,viewer_);
	cout<<"Cloud added"<<endl;
	addObjectToSpace(cloud_,cloud_output_,transformation_,viewer_);
	cout<<"Object added"<<endl;
	viewer_->addPointCloud (object_location, "locations");
	cout<<"Viewer Set"<<endl;
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
	cout<<"Dropdown set"<<endl;
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
	cout<<"Dropdown for axes done"<<endl;
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

	connect (model, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelChanged(QStandardItem*)));

	connect (model_axes, SIGNAL (itemChanged(QStandardItem*)), this, SLOT (modelAxesChanged(QStandardItem*)));

	connect (ui_->checkBox,  SIGNAL (clicked ()), this, SLOT (enableErrorCalculation()));
	cout<<"QT components loaded"<<endl;

	// Generate html for the table.
	tb_ = ui_->textBrowser;
	string htmlString = "<htmt><body><style>table, th, td {border: 1px solid black;}</style><center>Error Metrics</center><table><tr><th>Cloud Id</th><th>Avg Error</th><th>Max Error</th></tr>";
	QString html = QString::fromUtf8(htmlString.c_str());
	tb_->setHtml(html);
	pSliderValueChanged (2);
	viewer_->registerPointPickingCallback (pointPickingEventOccurred);
	ui_->qvtkWidget->update ();
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

void PCLViewer::updateErrorTable()
{
	int K = 1;
	string htmlString = "<!DOCTYPE html><htmt><body><style>table, th, td { border: 1px solid black;border-collapse: collapse;}</style><center>Error Metrics</center><table><tr><th>Cloud Id</th><th>Avg Error</th><th>Max Error</th></tr>";
	for(int j=0;j<clouds_.size();j++)
	{
		if(selected_clouds_[j]==false)
		{
			htmlString +="<tr><td>"+to_string(j)+"</td><td>"+"NAN"+"</td><td>NAN</td></tr>";
			continue;
		}
		float average = 0.0;
		float maximum = -1e9;
		int counter = 0;
		PointCloudT::Ptr temp(new PointCloudT);
		Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation_);
		world_T_object = world_T_object.inverse();
		transformPointCloud<pcl::PointXYZRGB>(cloud_outputs_[j],temp,world_T_object);
		cout<<temp->points.size()<<endl;
		for(int i=0;i<temp->points.size();i+=500)
		{
			counter++;
			pcl::PointXYZ searchPoint;
			searchPoint.x = temp->points[i].x;
			searchPoint.y = temp->points[i].y;
			searchPoint.z = temp->points[i].z;
			std::vector<int> pointIdxNKNSearch(K);
			std::vector<float> pointNKNSquaredDistance(K);
			if ( object_tree_.nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
			{
				float distance = sqrt(pointNKNSquaredDistance[0]);
				if(distance>maximum)
					maximum = distance;
				average+=distance;
			}
		}
		average=average/float(counter);
		htmlString +="<tr><td>"+to_string(j)+"</td><td>"+to_string(average)+"</td><td>"+to_string(maximum)+"</td></tr>";
	}
	htmlString+="</table></body></html>";
	QString html = QString::fromUtf8(htmlString.c_str());
	tb_->setHtml(html);
	cout<<"Done.."<<endl;
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

void PCLViewer::getInputs()
{

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

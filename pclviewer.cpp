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

void PCLViewer::enableErrorCalculation()
{
    calculate_error_=!calculate_error_;
    if(calculate_error_)
        updateErrorTable();
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
        if(selected_clouds_[selected_clouds_.size()-2])
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
    for(int i=2;i<selected_axes_.size()-1;i++)
        if(selected_axes_[i])
            addCoordinateAxes(inverse_kinematics_[i-2],viewer_,"flange"+to_string(i-1));
        else
            if(viewer_->contains("flange"+to_string(i-1)))
                viewer_->removeCoordinateSystem("flange"+to_string(i-1));

    if(selected_axes_[selected_axes_.size()-1])
    {
        if(viewer_->contains("origin"))
            viewer_->removeCoordinateSystem("origin");
        viewer_->addCoordinateSystem(1.0,Eigen::Affine3f::Identity(),"origin");
        cout<<"Adding the origin"<<endl;
    }
    else
    {
        if(viewer_->contains("origin"))
            viewer_->removeCoordinateSystem("origin");
        cout<<"Removing the origin"<<endl;
    }

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
    double max_error = -1e9;
    MatrixXf M = MatrixXf::Zero(3, cloud_->points.size());
    for(int i=0;i<cloud_->points.size();i++)
    {
        auto pt = cloud_->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    // #pragma omp parallel
    // #pragma omp for
    for(int j=0;j<clouds_.size();j++)
    {
        if(selected_clouds_[j]==false)
        {
            continue;
        }
        float average = 0.0;
        float maximum = -1e9;
        int counter = 0;
        // pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation_);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation_);
        Eigen::MatrixXd transformation = inverse_kinematics_[j]*cam_T_flange;
        world_T_object = world_T_object.inverse();
        // transformPointCloud<pcl::PointXYZ>(cloud_downsampled_[j],temp,transformation );
        MatrixXf N = MatrixXf::Zero(3, clouds_[j]->points.size());
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        Eigen::Affine3d transW;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                transW(a,b) = world_T_object(a,b);
        for(int i=0;i<clouds_[j]->points.size();i++)
        {
            auto point = clouds_[j]->points[i];
#if 1
            float src[3];
            float out[3];
            src[0] = point.x;
            src[1] = point.y;
            src[2] = point.z;
            apply_transformation_optimized(src,out,trans);
            src[0] = out[0];
            src[1] = out[1];
            src[2] = out[2];
            apply_transformation_optimized(src,out,transW);
            N(0,i) = out[0];
            N(1,i) = out[1];
            N(2,i) = out[2];
#else
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            pts=apply_transformation(pts,transformation);
            pts=apply_transformation(pts,world_T_object);
            N(0,i) = pts(0,0);
            N(1,i) = pts(0,1);
            N(2,i) = pts(0,2);
#endif
            counter++;
        }
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, N.cols());
        dists2.resize(1, N.cols());
        nns->knn(N, indices, dists2, 1, 0, NNSearchF::SORT_RESULTS);
        for(int i=0;i<counter;i++)
        {
            double distance = sqrt(dists2(0,i));
            if(maximum<distance)
                maximum=distance;
            average+=distance;
        }
        average=average/counter;
        std::cout<<average<<" "<<counter<<endl;
#if 0
        world_T_object = world_T_object.inverse();
        // transformPointCloud<pcl::PointXYZ>(temp,world_T_object);
        Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
        std::vector<int> pointIdxNKNSearch(K);
        std::vector<float> pointNKNSquaredDistance(K);
        std::vector<int> pointIdxNKNSearch2(K);
        std::vector<float> pointNKNSquaredDistance2(K);
        for(int i=0;i<cloud_downsampled_[j]->points.size();i+=1)
        {
            auto point = cloud_downsampled_[j]->points[i];
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            pts=apply_transformation(pts,transformation);
            pts=apply_transformation(pts,world_T_object);
            counter++;
            pcl::PointXYZ searchPoint;
            searchPoint.x = pts(0,0);
            searchPoint.y = pts(0,1);
            searchPoint.z = pts(0,2);
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
            if ( object_tree_vec2_[j].nearestKSearch (searchPoint, K, pointIdxNKNSearch2, pointNKNSquaredDistance2) > 0 )
            {
                float distance = sqrt(pointNKNSquaredDistance2[0]);
            }
            double error_now = fabs(sqrt(pointNKNSquaredDistance2[0])*1000.0-sqrt(pointNKNSquaredDistance[0])*1000.0);
            cout<<"Error: "<<error_now<<endl;
            cout<<"Values: "<<sqrt(pointNKNSquaredDistance2[0])<<" : "<<sqrt(pointNKNSquaredDistance[0])<<endl;
            if(max_error<error_now)
                max_error = error_now;
#endif
        }
#endif
        cout<<"Counted "<<counter<<" points."<<endl;
        // cout<<"Max Error : "<<max_error/1000.0<<endl;
        error_avg[j]=average;
        error_max[j]=maximum;
    }
    cout<<"Errors: "<<endl;
    for(int i=0;i<error_avg.size();i++)
    {
        cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
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
    ui_->label_30->setNum(value);
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

PCLViewer::~PCLViewer ()
{
    delete ui_;
}

vector<double> getTransVector(boost::property_tree::ptree &pt,string s)
{
    string angle_metric = pt.get<std::string>(s+".angle","radian");
    string camera_approx_trans_metric = pt.get<std::string>(s+".metric","m");
    double cam_approx_scale = 1.0;
    if(camera_approx_trans_metric=="cm")
        cam_approx_scale = 100.0;
    else if(camera_approx_trans_metric=="mm")
        cam_approx_scale=1000.0;
    string approx_transformation = pt.get<std::string>(s+".value","0,0,0,0,0,0");
    cout<<approx_transformation<<endl;
    vector<string> coords_str;
    boost::split(coords_str, approx_transformation , boost::is_any_of(","));
    vector<double> coords;
    for(int i=0;i<coords_str.size();i++)
        if(i<3)
            coords.push_back(stof(coords_str[i])/cam_approx_scale);
        else
        {
            double value = stof(coords_str[i]);
            if(angle_metric=="degree")
                value=degreeToRadian(value);
            coords.push_back(value);
        }

    for(auto x:coords)
        cout<<x<<" ";
    cout<<endl;
    return coords;
}

void PCLViewer::getInputs()
{
    // Reading the pointclouds, sensor scans and the transformations from files using XML config file.
    ifstream file;
    // file.open("/home/rex/REX_WS/Catkin_WS/src/CameraCalibration/config/config.xml");
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

void PCLViewer::pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void)
{
    std::cout << "[INOF] Point picking event occurred." << std::endl;
    float x, y, z;
    if (event.getPointIndex () == -1)
    {
        return;
    }
    event.getPoint(x, y, z);
    std::cout << "[INOF] Point coordinate ( " << x << ", " << y << ", " << z << ")" << std::endl;
    if(apply_svd_)
    {
        if(points_1_.size()<=points_2_.size())
        {
            points_1_.push_back({x,y,z});
        }
        else
        {
            points_2_.push_back({x,y,z});
        }

    }

}

void PCLViewer::algorithmSelected(int value)
{
    cout<<"Value: "<<value<<endl;
    algorithm_ = value;
    if(value==0)
    {
        cout<<"Printing the Values: "<<endl;
        if(points_1_.size()==points_2_.size())
            for(int i=0;i<points_1_.size();i++)
            {
                auto x = points_1_[i];
                auto y = points_2_[i];
                cout<<x[0]<<" "<<x[1]<<" "<<x[2]<<" : "<<y[0]<<" "<<y[1]<<" "<<y[2]<<endl;
            }
        points_1_.resize(0);
        points_2_.resize(0);

        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(this, "Test", "Use this result?",
                QMessageBox::Yes|QMessageBox::No);
        if (reply == QMessageBox::Yes) {
            cout << "Yes was clicked";
            for(int i=0;i<transformation_.size();i++)
            {
                transformation_initial_[i]=transformation_[i];
            }
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
        } else {
            cout << "Yes was *not* clicked";
            for(int i=0;i<transformation_.size();i++)
            {
                transformation_[i]=transformation_initial_[i];
            }
            updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,true);
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
        updateAxes();
        apply_svd_=false;
    }
    else if(value==1||value==2)
    {
        cout<<"Applying SVD"<<endl;
        apply_svd_=true;
    }
    else if(value==3)
    {
        //TODO: Optimizer code.
        cout<<"Applying optimizer"<<endl;
    }
}

void PCLViewer::applyAlgorithm()
{
    cout<<"Button Pressed"<<endl;
    if(algorithm_==1||algorithm_==2)
    {
        string filename="../scripts/input.tmp";
        ofstream file(filename);
        if(points_1_.size()!=points_2_.size())
            return;
        file<<points_1_.size()<<endl;
        //Input the global frame first.
        for(auto x:points_1_)
        {
            file<<x[0]<<","<<x[1]<<","<<x[2]<<endl;
        }

        for(auto x:points_2_)
        {
            file<<x[0]<<","<<x[1]<<","<<x[2]<<endl;
        }
        file.close();
        system("../scripts/svd.py");//TODO: Replace it with a C++ function.
        ifstream ifile("../scripts/output.tmp");
        string line;
        getline(ifile,line);
        vector<string> coords_str;
        boost::split(coords_str, line, boost::is_any_of(","));
        vector<double> coords;
        for(int i=0;i<coords_str.size();i++)
            coords.push_back(stof(coords_str[i]));
        for(int i=0;i<6;i++)
            if(i<3)
                cout<<coords[i]*1000.0<<" ";
            else
                cout<<radTodeg(coords[i])<<" ";
        cout<<endl;
        if(algorithm_==1)
        {
            updateObjectToSpace(cloud_,cloud_output_,coords,viewer_,true);
            for(int i=0;i<transformation_.size();i++)
                transformation_[i]=coords[i];

        }
        if(algorithm_==2)
        {
            int id = 0;
            for(;id<selected_clouds_.size()&&selected_clouds_[id]==false;id++);
            if(id==selected_clouds_.size())
                return;
            cout<<"Selected Cloud: "<<id<<endl;
            Eigen::MatrixXd cam_T_base = vectorToTransformationMatrix(coords);
            Eigen::MatrixXd inverse_kinematics = inverse_kinematics_[id];
            Eigen::MatrixXd cam_T_flange = inverse_kinematics.inverse()*cam_T_base; 
            Eigen::MatrixXd euler = rot2eul(cam_T_flange.block<3,3>(0,0),"");
            vector<double> transformation;
            for(int i=0;i<3;i++)
                transformation.push_back(cam_T_flange(0,i));
            for(int i=0;i<3;i++)
                transformation.push_back(euler(0,i));
            for(int i=0;i<6;i++)
                if(i<3)
                    cout<<transformation[i]*1000.0<<" ";
                else
                    cout<<radTodeg(transformation[i])<<" ";
            cout<<endl;
            updateClouds(clouds_,cloud_outputs_,inverse_kinematics_,transformation,viewer_,selected_clouds_);
        }
    }
    if(algorithm_==3)
    {
        findSeedPoints();
    }
}

void PCLViewer::findSeedPoints()
{
    int K = 1;
    
    MatrixXf M = MatrixXf::Zero(3, cloud_->points.size());
    for(int i=0;i<cloud_->points.size();i++)
    {
        auto pt = cloud_->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);

    vector<double> error_avg(clouds_.size(),1.0/0.0);
    vector<double> error_max(clouds_.size(),1.0/0.0);
    double max_error = -1e9;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    // #pragma omp parallel
    // #pragma omp for
    for(int j=0;j<clouds_.size();j++)
    {
        if(selected_clouds_[j]==false)
        {
            continue;
        }
        float average = 0.0;
        float maximum = -1e9;
        int counter = 0;
        Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation_);
        Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation_);
        Eigen::MatrixXd transformation = inverse_kinematics_[j]*cam_T_flange;
        world_T_object = world_T_object.inverse();
        MatrixXf N = MatrixXf::Zero(3, cloud_downsampled_[j]->points.size());
        Eigen::Affine3d trans;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                trans(a,b) = transformation(a,b);
        Eigen::Affine3d transW;
        for(int a=0;a<3;a++)
            for(int b=0;b<4;b++)
                transW(a,b) = world_T_object(a,b);
        for(int i=0;i<cloud_downsampled_[j]->points.size();i++)
        {
            auto point = cloud_downsampled_[j]->points[i];
#if 1
            float src[3];
            float out[3];
            src[0] = point.x;
            src[1] = point.y;
            src[2] = point.z;
            apply_transformation_optimized(src,out,trans);
            src[0] = out[0];
            src[1] = out[1];
            src[2] = out[2];
            apply_transformation_optimized(src,out,transW);
            N(0,i) = out[0];
            N(1,i) = out[1];
            N(2,i) = out[2];
#else
            pts(0,0)=point.x;
            pts(0,1)=point.y;
            pts(0,2)=point.z;
            pts=apply_transformation(pts,transformation);
            pts=apply_transformation(pts,world_T_object);
            N(0,i) = pts(0,0);
            N(1,i) = pts(0,1);
            N(2,i) = pts(0,2);
#endif
            counter++;
        }
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, N.cols());
        dists2.resize(1, N.cols());
        nns->knn(N, indices, dists2, 1, 0, NNSearchF::SORT_RESULTS);
        for(int i=0;i<counter;i++)
        {
            double distance = sqrt(dists2(0,i));
            if(maximum<distance)
                maximum=distance;
            average+=distance;
        }
        average=average/counter;
        std::cout<<average<<" "<<counter<<endl;
        cout<<"Counted "<<counter<<" points."<<endl;
        // cout<<"Max Error : "<<max_error/1000.0<<endl;
        error_avg[j]=average;
        error_max[j]=maximum;
    }
    
    cout<<"Errors: "<<endl;
    for(int i=0;i<error_avg.size();i++)
    {
        cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
    }
}

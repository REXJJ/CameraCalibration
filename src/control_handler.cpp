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

void PCLViewer::enableErrorCalculation()
{
    calculate_error_=!calculate_error_;
    if(calculate_error_)
        updateErrorTable();
}

void addCoordinateAxes(Eigen::MatrixXd& transformation,pcl::visualization::PCLVisualizer::Ptr viewer,string id)
{
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
                viewer_->removeCoordinateSystem("camera"+to_string(i+1));
        }
    }
    else
    {
        for(int i=0;i<clouds_.size();i++)
        {
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
            viewer_->removeCoordinateSystem("object");
        }
    }
    else
    {
        viewer_->removeCoordinateSystem("object");
    }
    for(int i=2;i<selected_axes_.size()-1;i++)
        if(selected_axes_[i])
            addCoordinateAxes(inverse_kinematics_[i-2],viewer_,"flange"+to_string(i-1));
        else
            viewer_->removeCoordinateSystem("flange"+to_string(i-1));

    if(selected_axes_[selected_axes_.size()-1])
    {
        viewer_->removeCoordinateSystem("origin");
        viewer_->addCoordinateSystem(1.0,Eigen::Affine3f::Identity(),"origin");
        cout<<"Adding the origin"<<endl;
    }
    else
    {
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
            updateSliders();
        } else {
            cout << "Yes was *not* clicked";
            for(int i=0;i<transformation_.size();i++)
            {
                transformation_[i]=transformation_initial_[i];
            }
            updateObjectToSpace(cloud_,cloud_output_,transformation_,viewer_,true);
            updateSliders();
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

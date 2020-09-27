#include "nabo/nabo.h"
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
#include <iostream>

using namespace Nabo;
using namespace Eigen;
using namespace std;
using namespace TransformationUtilities;
using namespace InputUtilities;
using namespace InterfaceUtilities;

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Optimizer
{
    private:
        string config_file;
        vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled;
        vector<MatrixXd> inverse_kinematics;
        std::vector<double> transformation;
        std::vector<double> flange_transformation;
        std::vector<double> transformation_initial;
        std::vector<double> flange_transformation_initial;
        vector<double> getTransVector(boost::property_tree::ptree &pt,string s);
    public:
        Optimizer(string filename);
        void getInputs();
        void discreteCombintorialOptimizerTranslation();
        void discreteCombintorialOptimizerRotation();
        void discreteCombintorialOptimizerSmallBruteForce();

};

Optimizer::Optimizer(string filename)
{
    config_file = filename;
    cloud.reset (new PointCloudT);
    transformation = std::vector<double>(6,0);
    flange_transformation = std::vector<double>(6,0);
    transformation_initial = std::vector<double>(6,0);
    flange_transformation_initial = std::vector<double>(6,0);
}

vector<double> Optimizer::getTransVector(boost::property_tree::ptree &pt,string s)
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
void Optimizer::getInputs()
{
    ifstream file;
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
	ifstream file(filename);
	string line;
	for(int c=0;c<14&&getline(file,line);c++);
        PointCloudT::Ptr pointcloud(new PointCloudT);
	while(getline(file,line))
	{
		vector<string> values_from_file;
		boost::split(values_from_file, line, boost::is_any_of(" "));
		pcl::PointXYZRGB pt;
		pt.x = stof(values_from_file[0]);
		pt.y = stof(values_from_file[1]);
		pt.z = stof(values_from_file[2]);
		pt.r = stof(values_from_file[3]);
		pt.g = stof(values_from_file[4]);
		pt.b = stof(values_from_file[5]);
		pointcloud->points.push_back(pt);
	}
        PointCloudT::Ptr temp_cloud(new PointCloudT);
        for(int i=0;i<pointcloud->points.size();i++)
        {
            auto pt = pointcloud->points[i];
            if(pt.x==0.0&&pt.y==0.0&&pt.z==0.0)
                continue;
            temp_cloud->points.push_back(pt);
        }
        clouds.push_back(temp_cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_bw_temp(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*temp_cloud,*cloud_bw_temp);
        pcl::VoxelGrid<pcl::PointXYZ> sor;
        sor.setInputCloud (cloud_bw_temp);
        constexpr double leaf = 0.0225f;
        // constexpr double leaf = 0.007f;
        sor.setLeafSize (leaf, leaf, leaf);
        sor.filter (*cloud_filtered);
        cloud_downsampled.push_back(cloud_filtered);
        // PointCloudT::Ptr pointcloud_output(new PointCloudT);
        // cloud_outputs.push_back(pointcloud_output);
        cout<<filename<<endl;
    }
    string ik_filename  = pt.get<std::string>("data.camera.transformations.inverse_kinematics");
    cout<<ik_filename<<endl;
    inverse_kinematics = readTransformations(ik_filename,true);
    cout<<"Transformations Read"<<endl;
    //Reading the scan cloud.
    string object_metric = pt.get<std::string>("data.scan.metric","m");
    cout<<"------------- "<<object_metric <<endl;
    for(const auto &cloud_location :pt.get_child("data.scan.clouds"))
    {
        string filename = cloud_location.second.data();
        readPointCloud(filename,cloud,object_metric );
    }
    cout<<"Scan read"<<endl; 
    //Reading Default Values;
    auto transformation_initial_object= getTransVector(pt,"data.scan.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_object.size();i++)
    {
        transformation[i]=transformation_initial_object[i];
        transformation_initial[i] = transformation_initial_object[i];
    }
    auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
    for(int i=0;i<transformation_initial_flange.size();i++)
    {
        flange_transformation[i]=transformation_initial_flange[i];
        flange_transformation_initial[i]=transformation_initial_flange[i];
    }
    cout<<"Here"<<endl;
}

void Optimizer::discreteCombintorialOptimizerTranslation()
{
    int K = 1;
    MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    //Setting up the structures.
    vector<MatrixXf> Ns;
    vector<MatrixXi> indis;
    vector<MatrixXf> dists;
    for(int i=0;i<clouds.size();i++)
    {
        MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[i]->points.size());
        Ns.push_back(N);
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, Ns[i].cols());
        dists2.resize(1, Ns[i].cols());
        indis.push_back(indices);
        dists.push_back(dists2);
        cout<<"Points Size: "<<cloud_downsampled[i]->points.size()<<endl;
    }
    double min_error = 1e9;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    vector<double> trans(6),flange_trans(6);
    int iter_min = -10;
    int iter_max = 10;
    TIC();
    int iterations = 0;
    for(int xf=iter_min;xf<=iter_max;xf+=2)
        for(int yf=iter_min;yf<=iter_max;yf+=2)
            for(int zf=iter_min;zf<=iter_max;zf+=2)
                for(int xo=iter_min;xo<=iter_max;xo+=2)
                    for(int yo=iter_min;yo<=iter_max;yo+=2)
                        for(int zo=iter_min;zo<=iter_max;zo+=2)
                        {
                            double err = 0.0;
                            flange_transformation[0]+=xf/1000.0;
                            flange_transformation[1]+=yf/1000.0;
                            flange_transformation[2]+=zf/1000.0;
                            transformation[0]+=xo/1000.0;
                            transformation[1]+=yo/1000.0;
                            transformation[2]+=zo/1000.0;
                            TIC();
                            // #pragma omp parallel
                            // #pragma omp for
                            for(int j=0;j<clouds.size();j++)
                            {
                                float average = 0.0;
                                float maximum = -1e9;
                                int counter = 0;
                                Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
                                Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
                                Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
                                world_T_object = world_T_object.inverse();
                                Eigen::Affine3d trans;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        trans(a,b) = transformation(a,b);
                                Eigen::Affine3d transW;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        transW(a,b) = world_T_object(a,b);
                                for(int i=0;i<cloud_downsampled[j]->points.size();i++)
                                {
                                    auto point = cloud_downsampled[j]->points[i];
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
                                    Ns[j](0,i) = out[0];
                                    Ns[j](1,i) = out[1];
                                    Ns[j](2,i) = out[2];
#else
                                    pts(0,0)=point.x;
                                    pts(0,1)=point.y;
                                    pts(0,2)=point.z;
                                    pts=apply_transformation(pts,transformation);
                                    pts=apply_transformation(pts,world_T_object);
                                    Ns[j](0,i) = pts(0,0);
                                    Ns[j](1,i) = pts(0,1);
                                    Ns[j](2,i) = pts(0,2);
#endif
                                    counter++;
                                }

                                nns->knn(Ns[j], indis[j],dists[j], 1, 0.1, NNSearchF::SORT_RESULTS);

                                for(int i=0;i<counter;i++)
                                {
                                    double distance = sqrt(dists[j](0,i));
                                    if(maximum<distance)
                                        maximum=distance;
                                    average+=distance;
                                }
                                average=average/counter;
                                err = err + average;
                            }
                            if(err<min_error)
                            {
                                min_error = err;
                                flange_trans = flange_transformation;
                                trans = transformation;
                            }
                            if(iterations%100000==0)
                            {
                                cout<<"Iteration: "<<iterations<<endl;
                                cout<<xf<<" "<<yf<<" "<<zf<<" "<<xo<<" "<<yo<<" "<<zo<<endl;
                                cout<<"Minimum Error: "<<min_error<<endl;
                                cout<<"Flange Transformation"<<endl;
                                for(auto x:flange_trans)
                                    cout<<x<<" ";
                                cout<<endl;
                                cout<<"Object Transformation"<<endl;
                                for(auto x:trans)
                                    cout<<x<<" ";
                                cout<<endl;
                            }
                            flange_transformation[0]-=xf/1000.0;
                            flange_transformation[1]-=yf/1000.0;
                            flange_transformation[2]-=zf/1000.0;
                            transformation[0]-=xo/1000.0;
                            transformation[1]-=yo/1000.0;
                            transformation[2]-=zo/1000.0;
                            iterations++;
                        }
    TOC();
    cout<<"Iterations: "<<iterations<<endl;
    cout<<"Minimum Error: "<<min_error<<endl;
    cout<<"Flange Transformation"<<endl;
    for(auto x:flange_trans)
        cout<<x<<" ";
    cout<<"Object Transformation"<<endl;
    for(auto x:trans)
        cout<<x<<" ";
}

void Optimizer::discreteCombintorialOptimizerRotation()
{
    int K = 1;
    MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    //Setting up the structures.
    vector<MatrixXf> Ns;
    vector<MatrixXi> indis;
    vector<MatrixXf> dists;
    for(int i=0;i<clouds.size();i++)
    {
        MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[i]->points.size());
        Ns.push_back(N);
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, Ns[i].cols());
        dists2.resize(1, Ns[i].cols());
        indis.push_back(indices);
        dists.push_back(dists2);
        cout<<"Points Size: "<<cloud_downsampled[i]->points.size()<<endl;
    }
    double min_error = 1e9;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    vector<double> trans(6),flange_trans(6);
    int iter_min = -4;
    int iter_max = 4;
    TIC();
    int iterations = 0;
    for(float xf=iter_min;xf<=iter_max;xf+=1)
        for(float yf=iter_min;yf<=iter_max;yf+=1)
            for(float zf=iter_min;zf<=iter_max;zf+=1)
                for(float xo=iter_min;xo<=iter_max;xo+=1)
                    for(float yo=iter_min;yo<=iter_max;yo+=1)
                        for(float zo=iter_min;zo<=iter_max;zo+=1)
                        {
                            double err = 0.0;
                            flange_transformation[3]+=degreeToRadian(xf/2.0);
                            flange_transformation[4]+=degreeToRadian(yf/2.0);
                            flange_transformation[5]+=degreeToRadian(zf/2.0);
                            transformation[3]+=degreeToRadian(xo/2.0);
                            transformation[4]+=degreeToRadian(yo/2.0);
                            transformation[5]+=degreeToRadian(zo/2.0);
                            TIC();
                            // #pragma omp parallel
                            // #pragma omp for
                            for(int j=0;j<clouds.size();j++)
                            {
                                float average = 0.0;
                                float maximum = -1e9;
                                int counter = 0;
                                Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
                                Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
                                Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
                                world_T_object = world_T_object.inverse();
                                Eigen::Affine3d trans;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        trans(a,b) = transformation(a,b);
                                Eigen::Affine3d transW;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        transW(a,b) = world_T_object(a,b);
                                for(int i=0;i<cloud_downsampled[j]->points.size();i++)
                                {
                                    auto point = cloud_downsampled[j]->points[i];
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
                                    Ns[j](0,i) = out[0];
                                    Ns[j](1,i) = out[1];
                                    Ns[j](2,i) = out[2];
#else
                                    pts(0,0)=point.x;
                                    pts(0,1)=point.y;
                                    pts(0,2)=point.z;
                                    pts=apply_transformation(pts,transformation);
                                    pts=apply_transformation(pts,world_T_object);
                                    Ns[j](0,i) = pts(0,0);
                                    Ns[j](1,i) = pts(0,1);
                                    Ns[j](2,i) = pts(0,2);
#endif
                                    counter++;
                                }

                                nns->knn(Ns[j], indis[j],dists[j], 1, 0.1, NNSearchF::SORT_RESULTS);

                                for(int i=0;i<counter;i++)
                                {
                                    double distance = sqrt(dists[j](0,i));
                                    if(maximum<distance)
                                        maximum=distance;
                                    average+=distance;
                                }
                                average=average/counter;
                                err = err + average*0.5 + maximum*0.5;
                            }
                            if(err<min_error)
                            {
                                min_error = err;
                                flange_trans = flange_transformation;
                                trans = transformation;
                            }
                            if(iterations%100000==0)
                            {
                                cout<<"Iteration: "<<iterations<<endl;
                                cout<<xf<<" "<<yf<<" "<<zf<<" "<<xo<<" "<<yo<<" "<<zo<<endl;
                                cout<<"Minimum Error: "<<min_error<<endl;
                                cout<<"Flange Transformation"<<endl;
                                for(auto x:flange_trans)
                                    cout<<x<<" ";
                                cout<<endl;
                                cout<<"Object Transformation"<<endl;
                                for(auto x:trans)
                                    cout<<x<<" ";
                                cout<<endl;
                            }
                            flange_transformation[3]-=degreeToRadian(xf/10.0);
                            flange_transformation[4]-=degreeToRadian(yf/10.0);
                            flange_transformation[5]-=degreeToRadian(zf/10.0);
                            transformation[3]-=degreeToRadian(xo/10.0);
                            transformation[4]-=degreeToRadian(yo/10.0);
                            transformation[5]-=degreeToRadian(zo/10.0);
                            iterations++;
                        }
    TOC();
    cout<<"Iterations: "<<iterations<<endl;
    cout<<"Minimum Error: "<<min_error<<endl;
    cout<<"Flange Transformation"<<endl;
    for(auto x:flange_trans)
        cout<<x<<" ";
    cout<<endl;
    cout<<"Object Transformation"<<endl;
    for(auto x:trans)
        cout<<x<<" ";
    cout<<endl;
    cout<<degreeToRadian(1)<<endl;
}

void Optimizer::discreteCombintorialOptimizerSmallBruteForce()
{
    int K = 1;
    MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
    for(int i=0;i<cloud->points.size();i++)
    {
        auto pt = cloud->points[i];
        M(0,i) = pt.x;
        M(1,i) = pt.y;
        M(2,i) = pt.z;
    }
    NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
    //Setting up the structures.
    vector<MatrixXf> Ns;
    vector<MatrixXi> indis;
    vector<MatrixXf> dists;
    for(int i=0;i<clouds.size();i++)
    {
        MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[i]->points.size());
        Ns.push_back(N);
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, Ns[i].cols());
        dists2.resize(1, Ns[i].cols());
        indis.push_back(indices);
        dists.push_back(dists2);
        cout<<"Points Size: "<<cloud_downsampled[i]->points.size()<<endl;
    }
    double min_error = 1e9;
    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    vector<double> trans(6),flange_trans(6);
    int iter_min = -1;
    int iter_max = 1;
    TIC();
    int iterations = 0;
    for(int x=0;x<7;x++)
        for(int a=iter_min;a<=iter_max;a++)
            for(int b=iter_min;b<=iter_max;b++)
                for(int c=iter_min;c<=iter_max;c++)
                    for(int d=iter_min;d<=iter_max;d++)
                        for(int e=iter_min;e<=iter_max;e++)
                            for(int f=iter_min;f<=iter_max;f++)
                                for(int g=iter_min;g<=iter_max;g++)
                                    for(int h=iter_min;h<=iter_max;h++)
                                        for(int i=iter_min;i<=iter_max;i++)
                                            for(int j=iter_min;j<=iter_max;j++)
                                                for(int k=iter_min;k<=iter_max;k++)
                                                    for(int l=iter_min;l<=iter_max;l++)
                                                    {
                                                        double err = 0.0;
                                                        flange_transformation[0]+=a/1000.0;
                                                        flange_transformation[1]+=b/1000.0;
                                                        flange_transformation[2]+=c/1000.0;
                                                        flange_transformation[3]+=degreeToRadian(float(d/2.0));
                                                        flange_transformation[4]+=degreeToRadian(float(e/2.0));
                                                        flange_transformation[5]+=degreeToRadian(float(f/2.0));
                                                        transformation[0]+=g/1000.0;
                                                        transformation[1]+=h/1000.0;
                                                        transformation[2]+=i/1000.0;
                                                        transformation[3]+=degreeToRadian(float(j/2.0));
                                                        transformation[4]+=degreeToRadian(float(k/2.0));
                                                        transformation[5]+=degreeToRadian(float(l/2.0));
                                                        TIC();
                                                        // #pragma omp parallel
                                                        // #pragma omp for
                                                        for(int j=0;j<clouds.size();j++)
                                                        {
                                                            float average = 0.0;
                                                            float maximum = -1e9;
                                                            int counter = 0;
                                                            Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
                                                            Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
                                                            Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
                                                            world_T_object = world_T_object.inverse();
                                                            Eigen::Affine3d trans;
                                                            for(int a=0;a<3;a++)
                                                                for(int b=0;b<4;b++)
                                                                    trans(a,b) = transformation(a,b);
                                                            Eigen::Affine3d transW;
                                                            for(int a=0;a<3;a++)
                                                                for(int b=0;b<4;b++)
                                                                    transW(a,b) = world_T_object(a,b);
                                                            for(int i=0;i<cloud_downsampled[j]->points.size();i++)
                                                            {
                                                                auto point = cloud_downsampled[j]->points[i];
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
                                                                Ns[j](0,i) = out[0];
                                                                Ns[j](1,i) = out[1];
                                                                Ns[j](2,i) = out[2];
#else
                                                                pts(0,0)=point.x;
                                                                pts(0,1)=point.y;
                                                                pts(0,2)=point.z;
                                                                pts=apply_transformation(pts,transformation);
                                                                pts=apply_transformation(pts,world_T_object);
                                                                Ns[j](0,i) = pts(0,0);
                                                                Ns[j](1,i) = pts(0,1);
                                                                Ns[j](2,i) = pts(0,2);
#endif
                                                                counter++;
                                                            }

                                                            nns->knn(Ns[j], indis[j],dists[j], 1, 0.1, NNSearchF::SORT_RESULTS);

                                                            for(int i=0;i<counter;i++)
                                                            {
                                                                double distance = sqrt(dists[j](0,i));
                                                                if(maximum<distance)
                                                                    maximum=distance;
                                                                average+=distance;
                                                            }
                                                            average=average/counter;
                                                            err = err + average*0.5 + maximum*0.5;
                                                        }
                                                        if(err<min_error)
                                                        {
                                                            min_error = err;
                                                            flange_trans = flange_transformation;
                                                            trans = transformation;
                                                        }
                                                        if(iterations%100000==0)
                                                        {
                                                            cout<<"Iteration: "<<iterations<<endl;
                                                            cout<<"Minimum Error: "<<min_error<<endl;
                                                            cout<<"Flange Transformation"<<endl;
                                                            for(auto x:flange_trans)
                                                                cout<<x<<" ";
                                                            cout<<endl;
                                                            cout<<"Object Transformation"<<endl;
                                                            for(auto x:trans)
                                                                cout<<x<<" ";
                                                            cout<<endl;
                                                        }
                                                        flange_transformation[0]-=a/1000.0;
                                                        flange_transformation[1]-=b/1000.0;
                                                        flange_transformation[2]-=c/1000.0;
                                                        flange_transformation[3]-=degreeToRadian(float(d/2.0));
                                                        flange_transformation[4]-=degreeToRadian(float(e/2.0));
                                                        flange_transformation[5]-=degreeToRadian(float(f/2.0));
                                                        transformation[0]-=g/1000.0;
                                                        transformation[1]-=h/1000.0;
                                                        transformation[2]-=i/1000.0;
                                                        transformation[3]-=degreeToRadian(float(j/2.0));
                                                        transformation[4]-=degreeToRadian(float(k/2.0));
                                                        transformation[5]-=degreeToRadian(float(l/2.0));
                                                        iterations++;
                                                    }
    TOC();
    cout<<"Iterations: "<<iterations<<endl;
    cout<<"Minimum Error: "<<min_error<<endl;
    cout<<"Flange Transformation"<<endl;
    for(auto x:flange_trans)
        cout<<x<<" ";
    cout<<endl;
    cout<<"Object Transformation"<<endl;
    for(auto x:trans)
        cout<<x<<" ";
    cout<<endl;
    cout<<degreeToRadian(1)<<endl;
}

int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    Optimizer opti = Optimizer(string(argv[1]));
    opti.getInputs();
    cout<<" Discrete optimization on Translation.."<<endl;
    // opti.discreteCombintorialOptimizerSmallBruteForce();
    opti.discreteCombintorialOptimizerTranslation();
    return 0;
}


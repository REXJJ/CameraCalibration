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


struct Object
{
    vector<PointCloudT::Ptr> clouds;
    vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled;
    PointCloudT::Ptr object;
    vector<MatrixXd> fks;
    vector<double> transformation;
    Object(vector<PointCloudT::Ptr> c,vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> d, PointCloudT::Ptr o,vector<MatrixXd> f,vector<double> t)
    {
        clouds=c;
        cloud_downsampled=d;
        object=o;
        cout<<object->points.size()<<endl;
        fks=f;
        transformation=t;
    }
};

class Optimizer
{
    private:
        vector<string> config_files;
        std::vector<double> flange_transformation;
        vector<Object> inputs;
        vector<double> getTransVector(boost::property_tree::ptree &pt,string s);
    public:
        Optimizer(vector<string> files);
        void getInputs();
        void discreteCombintorialOptimizerTranslation();

};

Optimizer::Optimizer(vector<string> filenames)
{
    config_files = filenames;
    flange_transformation = std::vector<double>(6,0);
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
    for(int f=0;f<config_files.size();f++)
    {
        vector<PointCloudT::Ptr> clouds;
        PointCloudT::Ptr cloud(new PointCloudT);
        vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled;
        vector<MatrixXd> inverse_kinematics;
        vector<double> transformation=std::vector<double>(6,0);

        ifstream file;
        string config_file = config_files[f];
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
        }
        auto transformation_initial_flange= getTransVector(pt,"data.camera.transformations.approximate_transformation");
        for(int i=0;i<transformation_initial_flange.size();i++)
        {
            flange_transformation[i]=transformation_initial_flange[i];
        }
        cout<<"Here"<<endl;
        inputs.push_back(Object(clouds,cloud_downsampled,cloud,inverse_kinematics,transformation));
    }
}

struct OptimizerParameters
{
    MatrixXf M;
    NNSearchF* nns;
    vector<MatrixXf> Ns;
    vector<MatrixXi> indis;
    vector<MatrixXf> dists;
    vector<double> trans_o;
    OptimizerParameters(MatrixXf m,NNSearchF* n,vector<MatrixXf> ns, vector<MatrixXi> in, vector<MatrixXf> d,vector<double> to)
    {
        M=m;
        nns=n;
        Ns=ns;
        indis=in;
        dists=d;
        trans_o = to;
    }
};



void Optimizer::discreteCombintorialOptimizerTranslation()
{
    constexpr int K = 1;
    int input = 0;
    MatrixXf M0 = MatrixXf::Zero(3, inputs[input].object->points.size());
    for(int i=0;i<inputs[input].object->points.size();i++)
    {
        auto pt = inputs[input].object->points[i];
        M0(0,i) = pt.x;
        M0(1,i) = pt.y;
        M0(2,i) = pt.z;
    }
    NNSearchF* nns0 = NNSearchF::createKDTreeLinearHeap(M0);
    //Setting up the structures.
    vector<MatrixXf> Ns0;
    vector<MatrixXi> indis0;
    vector<MatrixXf> dists0;
    for(int i=0;i<inputs[input].clouds.size();i++)
    {
        MatrixXf N = MatrixXf::Zero(3,inputs[input].cloud_downsampled[i]->points.size());
        Ns0.push_back(N);
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, Ns0[i].cols());
        dists2.resize(1, Ns0[i].cols());
        indis0.push_back(indices);
        dists0.push_back(dists2);
        cout<<"Points Size: "<<inputs[input].cloud_downsampled[i]->points.size()<<endl;
    }
    input = 1;
    MatrixXf M1 = MatrixXf::Zero(3, inputs[input].object->points.size());
    for(int i=0;i<inputs[input].object->points.size();i++)
    {
        auto pt = inputs[input].object->points[i];
        M1(0,i) = pt.x;
        M1(1,i) = pt.y;
        M1(2,i) = pt.z;
    }
    NNSearchF* nns1 = NNSearchF::createKDTreeLinearHeap(M1);
    //Setting up the structures.
    vector<MatrixXf> Ns1;
    vector<MatrixXi> indis1;
    vector<MatrixXf> dists1;
    for(int i=0;i<inputs[input].clouds.size();i++)
    {
        MatrixXf N = MatrixXf::Zero(3,inputs[input].cloud_downsampled[i]->points.size());
        Ns1.push_back(N);
        MatrixXi indices;
        MatrixXf dists2;
        indices.resize(1, Ns1[i].cols());
        dists2.resize(1, Ns1[i].cols());
        indis1.push_back(indices);
        dists1.push_back(dists2);
        cout<<"Points Size: "<<inputs[input].cloud_downsampled[i]->points.size()<<endl;
    }


    Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
    double min_error = 1e9;
    vector<double> flange_trans(6);
    int iter_min = -2;
    int iter_max = 2;
    TIC();
    int iterations = 0;
    vector<double>  optimal_transf_flange,optimal_trans_obj0,optimal_trans_obj1;

    for(int xf=iter_min;xf<=iter_max;xf+=2)
        for(int yf=iter_min;yf<=iter_max;yf+=2)
            for(int zf=iter_min;zf<=iter_max;zf+=2)
            {
                flange_transformation[0]+=xf/1000.0;
                flange_transformation[1]+=yf/1000.0;
                flange_transformation[2]+=zf/1000.0;

                vector<double> transformation_object0,transformation_object1;
                double error_min_1 = 1e9;
                double error_min_2 = 1e9;
                for(int xo=iter_min;xo<=iter_max;xo+=2)
                    for(int yo=iter_min;yo<=iter_max;yo+=2)
                        for(int zo=iter_min;zo<=iter_max;zo+=2)
                        {
                            int input = 0;
                            double err = 0.0;
                            auto transformation = inputs[input].transformation;
                            transformation[0]+=xo/1000.0;
                            transformation[1]+=yo/1000.0;
                            transformation[2]+=zo/1000.0;
                            auto clouds = inputs[input].clouds;
                            for(int j=0;j<clouds.size();j++)
                            {
                                float average = 0.0;
                                float maximum = -1e9;
                                int counter = 0;
                                Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
                                Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
                                Eigen::MatrixXd transformation = inputs[input].fks[j]*cam_T_flange;
                                world_T_object = world_T_object.inverse();
                                Eigen::Affine3d trans;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        trans(a,b) = transformation(a,b);
                                Eigen::Affine3d transW;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        transW(a,b) = world_T_object(a,b);
                                auto cloud_downsampled = inputs[input].cloud_downsampled;
                                for(int i=0;i<cloud_downsampled[j]->points.size();i++)
                                {
                                    auto point = cloud_downsampled[j]->points[i];

                                    counter++;
#if 0
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
                                    Ns0[j](0,i) = out[0];
                                    Ns0[j](1,i) = out[1];
                                    Ns0[j](2,i) = out[2];
#else
                                    pts(0,0)=point.x;
                                    pts(0,1)=point.y;
                                    pts(0,2)=point.z;
                                    pts=apply_transformation(pts,transformation);
                                    pts=apply_transformation(pts,world_T_object);
                                    Ns0[j](0,i) = pts(0,0);
                                    Ns0[j](1,i) = pts(0,1);
                                    Ns0[j](2,i) = pts(0,2);
#endif
                                }

                                nns0->knn(Ns0[j], indis0[j],dists0[j], 1, 0.1, NNSearchF::SORT_RESULTS);

                                for(int i=0;i<counter;i++)
                                {
                                    double distance = sqrt(dists0[j](0,i));
                                    if(maximum<distance)
                                        maximum=distance;
                                    average+=distance;
                                }
                                average=average/counter;
                                err+=average;
                            }
                            if(iterations%100000==0)
                            {
                                cout<<"Iterations: "<<iterations<<endl;
                                cout<<"Minimum Error: "<<min_error<<endl;
                                cout<<"Flange Transformation"<<endl;
                                for(auto x:optimal_transf_flange )
                                    cout<<x<<" ";
                                cout<<"Object Transformation"<<endl;
                                for(auto x:optimal_trans_obj0)
                                    cout<<x<<" ";
                                cout<<endl;
                                for(auto x:optimal_trans_obj1)
                                    cout<<x<<" ";
                                
                                ofstream ofile("temp_new.tmp",std::ios_base::app);
                                ofile<<"Iterations: "<<iterations<<endl;
                                ofile<<"Minimum Error: "<<min_error<<endl;
                                ofile<<"Flange Transformation"<<endl;
                                for(auto x:optimal_transf_flange )
                                    ofile<<x<<" ";
                                ofile<<"Object Transformation"<<endl;
                                for(auto x:optimal_trans_obj0)
                                    ofile<<x<<" ";
                                ofile<<endl;
                                for(auto x:optimal_trans_obj1)
                                    ofile<<x<<" ";
                            }
                            iterations+=1;
                            err=err/inputs[input].clouds.size();
                            if(err<error_min_1)
                            {
                                error_min_1=err;
                                transformation_object0 = transformation;
                            }
                        }

                for(int xo=iter_min;xo<=iter_max;xo+=2)
                    for(int yo=iter_min;yo<=iter_max;yo+=2)
                        for(int zo=iter_min;zo<=iter_max;zo+=2)
                        {
                            double err = 0.0;
                            auto transformation = inputs[input].transformation;
                            transformation[0]+=xo/1000.0;
                            transformation[1]+=yo/1000.0;
                            transformation[2]+=zo/1000.0;
                            int input = 1;
                            auto clouds = inputs[input].clouds;
                            for(int j=0;j<clouds.size();j++)
                            {
                                float average = 0.0;
                                float maximum = -1e9;
                                int counter = 0;
                                Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
                                Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
                                Eigen::MatrixXd transformation = inputs[input].fks[j]*cam_T_flange;
                                world_T_object = world_T_object.inverse();
                                Eigen::Affine3d trans;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        trans(a,b) = transformation(a,b);
                                Eigen::Affine3d transW;
                                for(int a=0;a<3;a++)
                                    for(int b=0;b<4;b++)
                                        transW(a,b) = world_T_object(a,b);
                                auto cloud_downsampled = inputs[input].cloud_downsampled;
                                for(int i=0;i<cloud_downsampled[j]->points.size();i++)
                                {
                                    auto point = cloud_downsampled[j]->points[i];
#if 0
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
                                    Ns1[j](0,i) = out[0];
                                    Ns1[j](1,i) = out[1];
                                    Ns1[j](2,i) = out[2];
#else
                                    pts(0,0)=point.x;
                                    pts(0,1)=point.y;
                                    pts(0,2)=point.z;
                                    pts=apply_transformation(pts,transformation);
                                    pts=apply_transformation(pts,world_T_object);
                                    Ns1[j](0,i) = pts(0,0);
                                    Ns1[j](1,i) = pts(0,1);
                                    Ns1[j](2,i) = pts(0,2);
#endif
                                    counter++;
                                }

                                nns1->knn(Ns1[j], indis1[j],dists1[j], 1, 0.1, NNSearchF::SORT_RESULTS);

                                for(int i=0;i<counter;i++)
                                {
                                    double distance = sqrt(dists1[j](0,i));
                                    if(maximum<distance)
                                        maximum=distance;
                                    average+=distance;
                                }
                                average=average/counter;
                                err+=average;
                            }
                            if(iterations%100000==0)
                            {
                                cout<<"Iterations: "<<iterations<<endl;
                                cout<<"Minimum Error: "<<min_error<<endl;
                                cout<<"Flange Transformation"<<endl;
                                for(auto x:optimal_transf_flange )
                                    cout<<x<<" ";
                                cout<<"Object Transformation"<<endl;
                                for(auto x:optimal_trans_obj0)
                                    cout<<x<<" ";
                                cout<<endl;
                                for(auto x:optimal_trans_obj1)
                                    cout<<x<<" ";
                                
                                ofstream ofile("temp_new.tmp",std::ios_base::app);
                                ofile<<"Iterations: "<<iterations<<endl;
                                ofile<<"Minimum Error: "<<min_error<<endl;
                                ofile<<"Flange Transformation"<<endl;
                                for(auto x:optimal_transf_flange )
                                    ofile<<x<<" ";
                                ofile<<"Object Transformation"<<endl;
                                for(auto x:optimal_trans_obj0)
                                    ofile<<x<<" ";
                                ofile<<endl;
                                for(auto x:optimal_trans_obj1)
                                    ofile<<x<<" ";
                            }
                            iterations+=1;
                            err=err/inputs[input].clouds.size();
                            if(err<error_min_2)
                            {
                                error_min_2=err;
                                transformation_object1 = transformation;
                            }

                            double error = (error_min_1+error_min_2)/2.0;
                            if(error<min_error)
                            {
                                min_error = error;
                                optimal_transf_flange = flange_transformation;
                                optimal_trans_obj0 = transformation_object0;
                                optimal_trans_obj1 = transformation_object1;
                            }

                        }

                flange_transformation[0]-=xf/1000.0;
                flange_transformation[1]-=yf/1000.0;
                flange_transformation[2]-=zf/1000.0;
            }
    TOC();
    cout<<"Iterations: "<<iterations<<endl;
    cout<<"Minimum Error: "<<min_error<<endl;
    cout<<"Flange Transformation"<<endl;
    for(auto x:optimal_transf_flange )
        cout<<x<<" ";
    cout<<"Object Transformation"<<endl;
    for(auto x:optimal_trans_obj0)
        cout<<x<<" ";
    cout<<endl;
    for(auto x:optimal_trans_obj1)
        cout<<x<<" ";
    cout<<endl;
    for(auto x:inputs[0].transformation)
        cout<<x<<" ";
    cout<<endl;
    for(auto x:inputs[1].transformation)
        cout<<x<<" ";
}


int main(int argc, char** argv)
{
    if(argc<2)
    {
        std::cout<<"Usage: optimizer_test <config file>"<<endl;
        exit(-1);
    }
    vector<string> files;
    for(int i=1;i<argc;i++)
        files.push_back(argv[i]);
    Optimizer opti(files);
    opti.getInputs();
    cout<<" Discrete optimization on Translation.."<<endl;
    // opti.discreteCombintorialOptimizerSmallBruteForce();
    opti.discreteCombintorialOptimizerTranslation();
    return 0;
}


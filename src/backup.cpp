
// void Optimizer::findSeedPoints()
// {
//     int K = 1;
//     MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//     for(int i=0;i<cloud->points.size();i++)
//     {
//         auto pt = cloud->points[i];
//         M(0,i) = pt.x;
//         M(1,i) = pt.y;
//         M(2,i) = pt.z;
//     }
//     NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//
//     vector<double> error_avg(clouds.size(),1.0/0.0);
//     vector<double> error_max(clouds.size(),1.0/0.0);
//     double max_error = -1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     // #pragma omp parallel
//     // #pragma omp for
//     TIC();
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             N(0,i) = out[0];
//             N(1,i) = out[1];
//             N(2,i) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//             counter++;
//         }
//         MatrixXi indices;
//         MatrixXf dists2;
//         indices.resize(1, N.cols());
//         dists2.resize(1, N.cols());
//         nns->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);
//
//         for(int i=0;i<counter;i++)
//         {
//             double distance = sqrt(dists2(0,i));
//             if(maximum<distance)
//                 maximum=distance;
//             average+=distance;
//         }
//         average=average/counter;
//         error_avg[j]=average;
//         error_max[j]=maximum;
//     }
//     TOC();
//     cout<<"Errors: "<<endl;
//     for(int i=0;i<error_avg.size();i++)
//     {
//         cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
//     }
// }
// void Optimizer::findSeedPointsPar()
// {
//     int K = 1;
//     vector<NNSearchF*> nabos;
//     vector<MatrixXf> mats;
//     for(int i=0;i<clouds.size();i++)
//     {
//         MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//         for(int i=0;i<cloud->points.size();i++)
//         {
//             auto pt = cloud->points[i];
//             M(0,i) = pt.x;
//             M(1,i) = pt.y;
//             M(2,i) = pt.z;
//         }
//         NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//         mats.push_back(M);
//         nabos.push_back(nns);
//     }
//     vector<double> error_avg(clouds.size(),1.0/0.0);
//     vector<double> error_max(clouds.size(),1.0/0.0);
//     double max_error = -1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     TIC();
//     #pragma omp parallel
//     #pragma omp for
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             N(0,i) = out[0];
//             N(1,i) = out[1];
//             N(2,i) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//             counter++;
//         }
//         MatrixXi indices;
//         MatrixXf dists2;
//         indices.resize(1, N.cols());
//         dists2.resize(1, N.cols());
//         nabos[j]->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);
//         for(int i=0;i<counter;i++)
//         {
//             double distance = sqrt(dists2(0,i));
//             if(maximum<distance)
//                 maximum=distance;
//             average+=distance;
//         }
//         average=average/counter;
//         error_avg[j]=average;
//         error_max[j]=maximum;
//     }
//     TOC();
//     cout<<"Errors: "<<endl;
//     for(int i=0;i<error_avg.size();i++)
//     {
//         cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
//     }
// }
// void Optimizer::findSeedPointsParallel()
// {
//     int K = 1;
//     vector<NNSearchF*> nabos;
//     vector<MatrixXf> mats;
//     for(int i=0;i<clouds.size();i++)
//     {
//         MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//         for(int i=0;i<cloud->points.size();i++)
//         {
//             auto pt = cloud->points[i];
//             M(0,i) = pt.x;
//             M(1,i) = pt.y;
//             M(2,i) = pt.z;
//         }
//         NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//         mats.push_back(M);
//         nabos.push_back(nns);
//     }
//     
//     vector<double> error_avg(clouds.size(),1.0/0.0);
//     vector<double> error_max(clouds.size(),1.0/0.0);
//     double max_error = -1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     TIC();
//     #pragma omp parallel
//     #pragma omp for
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         VectorXf q = VectorXf::Zero(3);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             q(0) = out[0];
//             q(1) = out[1];
//             q(2) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//         
//         VectorXi indices(K);
//         VectorXf dists2(K);
//         nabos[j]->knn(q, indices, dists2, K,0.1,NNSearchF::SORT_RESULTS);
//         double distance = sqrt(dists2(0));
//         if(maximum<distance)
//             maximum=distance;
//         average+=distance;
//         counter++;
//         }
//         average=average/counter;
//         error_avg[j]=average;
//         error_max[j]=maximum;
//     }
//     TOC();
//     cout<<"Errors: "<<endl;
//     for(int i=0;i<error_avg.size();i++)
//     {
//         cout<<"Error Average: "<<error_avg[i]<<" Error Max: "<<error_max[i]<<endl;
//     }
//     // delete nns;
// }

//     int K = 1;
//     cout<<"Starting Optimization. "<<endl;
//     MatrixXf M = MatrixXf::Zero(3, cloud->points.size());
//     for(int i=0;i<cloud->points.size();i++)
//     {
//         auto pt = cloud->points[i];
//         M(0,i) = pt.x;
//         M(1,i) = pt.y;
//         M(2,i) = pt.z;
//     }
//     NNSearchF* nns = NNSearchF::createKDTreeLinearHeap(M);
//     double min_error = 1e9;
//     Eigen::MatrixXd pts=Eigen::MatrixXd::Zero(1,3);
//     vector<double> trans(6),flange_trans(6);
//     int iter_min = -2;
//     int iter_max = 2;
//     // for(int xf=iter_min;xf<=iter_max;xf+=2)
//     // for(int yf=iter_min;yf<=iter_max;yf+=2)
//     // for(int zf=iter_min;zf<=iter_max;zf+=2)
//     // for(int xo=iter_min;xo<=iter_max;xo+=2)
//     // for(int yo=iter_min;yo<=iter_max;yo+=2)
//     // for(int zo=iter_min;zo<=iter_max;zo+=2)
//     // {
//     double err = 0.0;
//     // flange_transformation[0]+=xf/1000.0;
//     // flange_transformation[1]+=yf/1000.0;
//     // flange_transformation[2]+=zf/1000.0;
//     // transformation[0]+=xo/1000.0;
//     // transformation[1]+=yo/1000.0;
//     // transformation[2]+=zo/1000.0;
//     for(int j=0;j<clouds.size();j++)
//     {
//         float average = 0.0;
//         float maximum = -1e9;
//         int counter = 0;
//         Eigen::MatrixXd world_T_object = vectorToTransformationMatrix(transformation);
//         Eigen::MatrixXd cam_T_flange = vectorToTransformationMatrix(flange_transformation);
//         Eigen::MatrixXd transformation = inverse_kinematics[j]*cam_T_flange;
//         world_T_object = world_T_object.inverse();
//         MatrixXf N = MatrixXf::Zero(3, cloud_downsampled[j]->points.size());
//         Eigen::Affine3d trans;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 trans(a,b) = transformation(a,b);
//         Eigen::Affine3d transW;
//         for(int a=0;a<3;a++)
//             for(int b=0;b<4;b++)
//                 transW(a,b) = world_T_object(a,b);
//         for(int i=0;i<cloud_downsampled[j]->points.size();i++)
//         {
//             auto point = cloud_downsampled[j]->points[i];
// #if 1
//             float src[3];
//             float out[3];
//             src[0] = point.x;
//             src[1] = point.y;
//             src[2] = point.z;
//             apply_transformation_optimized(src,out,trans);
//             src[0] = out[0];
//             src[1] = out[1];
//             src[2] = out[2];
//             apply_transformation_optimized(src,out,transW);
//             N(0,i) = out[0];
//             N(1,i) = out[1];
//             N(2,i) = out[2];
// #else
//             pts(0,0)=point.x;
//             pts(0,1)=point.y;
//             pts(0,2)=point.z;
//             pts=apply_transformation(pts,transformation);
//             pts=apply_transformation(pts,world_T_object);
//             N(0,i) = pts(0,0);
//             N(1,i) = pts(0,1);
//             N(2,i) = pts(0,2);
// #endif
//             counter++;
//         }
//         MatrixXi indices;
//         MatrixXf dists2;
//         indices.resize(1, N.cols());
//         dists2.resize(1, N.cols());
//         nns->knn(N, indices, dists2, 1, 0.1, NNSearchF::SORT_RESULTS);        for(int i=0;i<counter;i++)
//         {
//             double distance = sqrt(dists2(0,i));
//             if(maximum<distance)
//                 maximum=distance;
//             average+=distance;
//         }
//         average=average/counter;
//         // err = err + average*0.5 + maximum*0.5;
//     }
//     // if(err<min_error)
//     // {
//     //     min_error = err;
//     //     flange_trans = flange_transformation;
//     //     trans = transformation;
//     // }
//     // flange_transformation[0]-=xf/1000.0;
//     // flange_transformation[1]-=yf/1000.0;
//     // flange_transformation[2]-=zf/1000.0;
//     // transformation[0]-=xo/1000.0;
//     // transformation[1]-=yo/1000.0;
//     // transformation[2]-=zo/1000.0;
//     // }
//     cout<<"Min Error: "<<min_error<<endl;
//     cout<<"Flange Transformation"<<endl;
//     for(auto x:flange_trans)
//         cout<<x<<" ";
//     cout<<"Object Transformation"<<endl;
//     for(auto x:trans)
//         cout<<x<<" ";
//     delete nns;
//

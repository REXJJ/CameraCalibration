#pragma once

#include <iostream>
#include <unordered_map>
// Qt
#include <QMainWindow>
#include <QTextBrowser>
#include <QStandardItemModel>
#include <QSlider>
#include <QLabel>
#include <QMessageBox>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/features/normal_3d.h>
// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>
#include "helpers.hpp"
#include "nabo/nabo.h"

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

using namespace Nabo;

namespace Ui
{
    class PCLViewer;
}

class PCLViewer : public QMainWindow
{
    Q_OBJECT

    public:
        explicit PCLViewer (QWidget *parent = 0);
        ~PCLViewer ();

        public Q_SLOTS:
            // void
            // randomButtonPressed ();

            void objectSliderReleased ();
            void cameraSliderReleased ();

        void pSliderValueChanged (int value);

        void zThreshSliderValueChanged (int value);

        void xSliderValueChanged (int value);

        void ySliderValueChanged (int value);

        void zSliderValueChanged (int value);

        void dxSliderValueChanged (int value);

        void dySliderValueChanged (int value);

        void dzSliderValueChanged (int value);
        
        void xCameraSliderValueChanged (int value);

        void yCameraSliderValueChanged (int value);

        void zCameraSliderValueChanged (int value);

        void dxCameraSliderValueChanged (int value);

        void dyCameraSliderValueChanged (int value);

        void dzCameraSliderValueChanged (int value);

        void modelChanged (QStandardItem *item);

        void modelAxesChanged (QStandardItem *item);

        void updateAxes();

        void enableErrorCalculation();

        void updateErrorTable();

        void transResolutionChanged(int value);

        void rotResolutionChanged(int value);

        void algorithmSelected(int value);

        void applyAlgorithm();

        void pointPickingEventOccurred (const pcl::visualization::PointPickingEvent& event, void* viewer_void);

        void enablePointsClassifier();


    protected:
        pcl::visualization::PCLVisualizer::Ptr viewer_;
        PointCloudT::Ptr cloud_;
        PointCloudT::Ptr cloud_output_;
        PointCloudT::Ptr object_location_;
        QTextBrowser *tb_;
        std::vector<double> transformation_;
        std::vector<double> flange_transformation_;
        std::vector<double> transformation_initial_;
        std::vector<double> flange_transformation_initial_;
        std::vector<PointCloudT::Ptr> clouds_;
        std::vector<PointCloudT::Ptr> cloud_outputs_;
        std::vector<PointCloudT::Ptr> cloud_classified_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled_;
        std::vector<Eigen::MatrixXd> inverse_kinematics_;
        std::vector<QStandardItem*> items_; 
        std::vector<bool> selected_clouds_;
        std::vector<bool> selected_axes_;
        bool calculate_error_;
        pcl::KdTreeFLANN<pcl::PointXYZ> object_tree_; 
        std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> object_tree_vec_; 
        std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> object_tree_vec2_; 
        std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> object_kdtree_vec_;  
        double translation_resolution_;
        double rotation_resolution_;
        std::vector<std::vector<double>> points_1_;//Points in the target frame
        std::vector<std::vector<double>> points_2_;
        std::vector<NNSearchF*> nabos_;
        NNSearchF* kd_tree_nabo_;
        bool apply_svd_;
        bool identify_good_points_;
        int z_threshold_;
        bool use_plane_;
        vector<double> plane_;
        unordered_map<int,int> mapping_;

    private:
        Ui::PCLViewer *ui_;
        QStandardItemModel *model_clouds_;
        QStandardItemModel *model_axes_;
        int algorithm_;
        void getInputs();
        void setupViz();
        void setupInterface();
        void addWidgets();
        double updateTranslationControls(QSlider*,QLabel*,QLabel*,double);
        double  updateRotationControls(QSlider*,QLabel*,QLabel*,double);
        void findSeedPoints();
        void classifyPoints();
        void updateSliders();
        void updateClouds(vector<PointCloudT::Ptr> clouds={});
        void showErrorsInPoints();
};

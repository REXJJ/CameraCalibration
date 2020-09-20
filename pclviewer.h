#pragma once

#include <iostream>
// Qt
#include <QMainWindow>
#include <QTextBrowser>
#include <QStandardItemModel>
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

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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

    protected:
        pcl::visualization::PCLVisualizer::Ptr viewer_;
        PointCloudT::Ptr cloud_;
        PointCloudT::Ptr cloud_output_;
        PointCloudT::Ptr object_location_;
        QTextBrowser *tb_;
        std::vector<double> transformation_;
        std::vector<double> flange_transformation_;
        std::vector<PointCloudT::Ptr> clouds_;
        std::vector<PointCloudT::Ptr> cloud_outputs_;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_downsampled_;
        std::vector<Eigen::MatrixXd> inverse_kinematics_;
        std::vector<QStandardItem*> items_; 
        std::vector<bool> selected_clouds_;
        std::vector<bool> selected_axes_;
        bool calculate_error_;
        pcl::KdTreeFLANN<pcl::PointXYZ> object_tree_; 
        std::vector<pcl::KdTreeFLANN<pcl::PointXYZ>> object_tree_vec_; 
        std::vector<pcl::search::KdTree<pcl::PointXYZ>::Ptr> object_kdtree_vec_;  

    private:
        Ui::PCLViewer *ui_;
        QStandardItemModel *model_clouds_;
        QStandardItemModel *model_axes_;
        void getInputs();
        void setupViz();
        void setupInterface();
        void addWidgets();
};

#pragma once

#include <iostream>
// Qt
#include <QMainWindow>
#include <QTextBrowser>
// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
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

            void RGBsliderReleased ();

        void pSliderValueChanged (int value);

        void xSliderValueChanged (int value);

        void ySliderValueChanged (int value);

        void zSliderValueChanged (int value);

        void dxSliderValueChanged (int value);

        void dySliderValueChanged (int value);

        void dzSliderValueChanged (int value);

    protected:
        pcl::visualization::PCLVisualizer::Ptr viewer_;
        PointCloudT::Ptr cloud_;
        PointCloudT::Ptr cloud_output_;
        QTextBrowser *tb_;
        std::vector<double> transformation_;

    private:
        Ui::PCLViewer *ui_;

};

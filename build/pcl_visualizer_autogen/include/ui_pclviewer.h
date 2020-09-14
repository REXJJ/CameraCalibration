/********************************************************************************
** Form generated from reading UI file 'pclviewer.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PCLVIEWER_H
#define UI_PCLVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QSlider>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>
#include "QVTKWidget.h"

QT_BEGIN_NAMESPACE

class Ui_PCLViewer
{
public:
    QWidget *centralwidget;
    QVTKWidget *qvtkWidget;
    QSlider *horizontalSlider_p;
    QLCDNumber *lcdNumber_p;
    QLabel *label_4;
    QGroupBox *groupBox;
    QLabel *label;
    QLCDNumber *lcdNumber_x;
    QSlider *horizontalSlider_X_Object;
    QLabel *label_2;
    QLCDNumber *lcdNumber_y;
    QSlider *horizontalSlider_Y_Object;
    QLabel *label_3;
    QSlider *horizontalSlider_Z_Object;
    QLCDNumber *lcdNumber_z;
    QSlider *horizontalSlider_dx_Object;
    QLCDNumber *lcdNumber_dx;
    QLabel *label_6;
    QSlider *horizontalSlider_dy_Object;
    QLCDNumber *lcdNumber_dy;
    QLabel *label_7;
    QLabel *label_8;
    QSlider *horizontalSlider_dz_Object;
    QLCDNumber *lcdNumber_dz;
    QGroupBox *groupBox_2;
    QLabel *label_15;
    QLCDNumber *lcdNumber_x_3;
    QSlider *horizontalSlider_X_Camera;
    QLabel *label_16;
    QLCDNumber *lcdNumber_y_3;
    QSlider *horizontalSlider_Y_Camera;
    QLabel *label_17;
    QSlider *horizontalSlider_Z_Camera;
    QLCDNumber *lcdNumber_z_3;
    QSlider *horizontalSlider_dx_Camera;
    QLCDNumber *lcdNumber_dx_3;
    QLabel *label_18;
    QSlider *horizontalSlider_dy_Camera;
    QLCDNumber *lcdNumber_dy_3;
    QLabel *label_19;
    QLabel *label_20;
    QSlider *horizontalSlider_dz_Camera;
    QLCDNumber *lcdNumber_dz_3;
    QTextBrowser *textBrowser;
    QListView *listView;

    void setupUi(QMainWindow *PCLViewer)
    {
        if (PCLViewer->objectName().isEmpty())
            PCLViewer->setObjectName(QStringLiteral("PCLViewer"));
        PCLViewer->resize(1620, 816);
        PCLViewer->setMinimumSize(QSize(0, 0));
        PCLViewer->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(PCLViewer);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        qvtkWidget = new QVTKWidget(centralwidget);
        qvtkWidget->setObjectName(QStringLiteral("qvtkWidget"));
        qvtkWidget->setGeometry(QRect(320, 40, 1001, 631));
        QFont font;
        font.setPointSize(11);
        qvtkWidget->setFont(font);
        horizontalSlider_p = new QSlider(centralwidget);
        horizontalSlider_p->setObjectName(QStringLiteral("horizontalSlider_p"));
        horizontalSlider_p->setGeometry(QRect(50, 740, 160, 29));
        horizontalSlider_p->setMinimum(1);
        horizontalSlider_p->setMaximum(6);
        horizontalSlider_p->setValue(2);
        horizontalSlider_p->setOrientation(Qt::Horizontal);
        lcdNumber_p = new QLCDNumber(centralwidget);
        lcdNumber_p->setObjectName(QStringLiteral("lcdNumber_p"));
        lcdNumber_p->setGeometry(QRect(220, 740, 51, 21));
        lcdNumber_p->setDigitCount(1);
        lcdNumber_p->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_p->setProperty("intValue", QVariant(2));
        label_4 = new QLabel(centralwidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(50, 710, 141, 31));
        QFont font1;
        font1.setPointSize(10);
        font1.setBold(false);
        font1.setItalic(false);
        font1.setWeight(50);
        label_4->setFont(font1);
        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        groupBox->setGeometry(QRect(40, 30, 261, 301));
        label = new QLabel(groupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(10, 30, 191, 31));
        label->setFont(font1);
        lcdNumber_x = new QLCDNumber(groupBox);
        lcdNumber_x->setObjectName(QStringLiteral("lcdNumber_x"));
        lcdNumber_x->setGeometry(QRect(180, 50, 61, 21));
        lcdNumber_x->setSmallDecimalPoint(false);
        lcdNumber_x->setDigitCount(4);
        lcdNumber_x->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_x->setProperty("value", QVariant(0));
        lcdNumber_x->setProperty("intValue", QVariant(0));
        horizontalSlider_X_Object = new QSlider(groupBox);
        horizontalSlider_X_Object->setObjectName(QStringLiteral("horizontalSlider_X_Object"));
        horizontalSlider_X_Object->setGeometry(QRect(10, 50, 160, 29));
        horizontalSlider_X_Object->setMinimum(-255);
        horizontalSlider_X_Object->setMaximum(255);
        horizontalSlider_X_Object->setValue(0);
        horizontalSlider_X_Object->setSliderPosition(0);
        horizontalSlider_X_Object->setOrientation(Qt::Horizontal);
        label_2 = new QLabel(groupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(10, 70, 191, 31));
        label_2->setFont(font1);
        lcdNumber_y = new QLCDNumber(groupBox);
        lcdNumber_y->setObjectName(QStringLiteral("lcdNumber_y"));
        lcdNumber_y->setGeometry(QRect(180, 90, 61, 21));
        lcdNumber_y->setDigitCount(4);
        lcdNumber_y->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_y->setProperty("value", QVariant(0));
        lcdNumber_y->setProperty("intValue", QVariant(0));
        horizontalSlider_Y_Object = new QSlider(groupBox);
        horizontalSlider_Y_Object->setObjectName(QStringLiteral("horizontalSlider_Y_Object"));
        horizontalSlider_Y_Object->setGeometry(QRect(10, 90, 160, 29));
        horizontalSlider_Y_Object->setMinimum(-255);
        horizontalSlider_Y_Object->setMaximum(255);
        horizontalSlider_Y_Object->setValue(0);
        horizontalSlider_Y_Object->setOrientation(Qt::Horizontal);
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(10, 110, 191, 31));
        label_3->setFont(font1);
        horizontalSlider_Z_Object = new QSlider(groupBox);
        horizontalSlider_Z_Object->setObjectName(QStringLiteral("horizontalSlider_Z_Object"));
        horizontalSlider_Z_Object->setGeometry(QRect(10, 130, 160, 29));
        horizontalSlider_Z_Object->setMinimum(-255);
        horizontalSlider_Z_Object->setMaximum(256);
        horizontalSlider_Z_Object->setValue(0);
        horizontalSlider_Z_Object->setOrientation(Qt::Horizontal);
        lcdNumber_z = new QLCDNumber(groupBox);
        lcdNumber_z->setObjectName(QStringLiteral("lcdNumber_z"));
        lcdNumber_z->setGeometry(QRect(180, 130, 61, 21));
        lcdNumber_z->setDigitCount(4);
        lcdNumber_z->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_z->setProperty("value", QVariant(0));
        lcdNumber_z->setProperty("intValue", QVariant(0));
        horizontalSlider_dx_Object = new QSlider(groupBox);
        horizontalSlider_dx_Object->setObjectName(QStringLiteral("horizontalSlider_dx_Object"));
        horizontalSlider_dx_Object->setGeometry(QRect(10, 170, 160, 29));
        horizontalSlider_dx_Object->setMinimum(-180);
        horizontalSlider_dx_Object->setMaximum(180);
        horizontalSlider_dx_Object->setValue(0);
        horizontalSlider_dx_Object->setOrientation(Qt::Horizontal);
        lcdNumber_dx = new QLCDNumber(groupBox);
        lcdNumber_dx->setObjectName(QStringLiteral("lcdNumber_dx"));
        lcdNumber_dx->setGeometry(QRect(180, 170, 61, 21));
        lcdNumber_dx->setDigitCount(4);
        lcdNumber_dx->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_dx->setProperty("intValue", QVariant(0));
        label_6 = new QLabel(groupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        label_6->setGeometry(QRect(10, 150, 191, 31));
        label_6->setFont(font1);
        horizontalSlider_dy_Object = new QSlider(groupBox);
        horizontalSlider_dy_Object->setObjectName(QStringLiteral("horizontalSlider_dy_Object"));
        horizontalSlider_dy_Object->setGeometry(QRect(10, 210, 160, 29));
        horizontalSlider_dy_Object->setMinimum(-180);
        horizontalSlider_dy_Object->setMaximum(180);
        horizontalSlider_dy_Object->setValue(0);
        horizontalSlider_dy_Object->setOrientation(Qt::Horizontal);
        lcdNumber_dy = new QLCDNumber(groupBox);
        lcdNumber_dy->setObjectName(QStringLiteral("lcdNumber_dy"));
        lcdNumber_dy->setGeometry(QRect(180, 210, 61, 21));
        lcdNumber_dy->setDigitCount(4);
        lcdNumber_dy->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_dy->setProperty("value", QVariant(0));
        lcdNumber_dy->setProperty("intValue", QVariant(0));
        label_7 = new QLabel(groupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        label_7->setGeometry(QRect(10, 190, 191, 31));
        label_7->setFont(font1);
        label_8 = new QLabel(groupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        label_8->setGeometry(QRect(10, 230, 191, 31));
        label_8->setFont(font1);
        horizontalSlider_dz_Object = new QSlider(groupBox);
        horizontalSlider_dz_Object->setObjectName(QStringLiteral("horizontalSlider_dz_Object"));
        horizontalSlider_dz_Object->setGeometry(QRect(10, 250, 160, 29));
        horizontalSlider_dz_Object->setMinimum(-180);
        horizontalSlider_dz_Object->setMaximum(180);
        horizontalSlider_dz_Object->setValue(0);
        horizontalSlider_dz_Object->setOrientation(Qt::Horizontal);
        lcdNumber_dz = new QLCDNumber(groupBox);
        lcdNumber_dz->setObjectName(QStringLiteral("lcdNumber_dz"));
        lcdNumber_dz->setGeometry(QRect(180, 250, 61, 21));
        lcdNumber_dz->setSmallDecimalPoint(false);
        lcdNumber_dz->setDigitCount(4);
        lcdNumber_dz->setMode(QLCDNumber::Dec);
        lcdNumber_dz->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_dz->setProperty("value", QVariant(0));
        lcdNumber_dz->setProperty("intValue", QVariant(0));
        groupBox_2 = new QGroupBox(centralwidget);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        groupBox_2->setGeometry(QRect(40, 370, 261, 301));
        label_15 = new QLabel(groupBox_2);
        label_15->setObjectName(QStringLiteral("label_15"));
        label_15->setGeometry(QRect(10, 30, 191, 31));
        label_15->setFont(font1);
        lcdNumber_x_3 = new QLCDNumber(groupBox_2);
        lcdNumber_x_3->setObjectName(QStringLiteral("lcdNumber_x_3"));
        lcdNumber_x_3->setGeometry(QRect(180, 50, 61, 21));
        lcdNumber_x_3->setSmallDecimalPoint(false);
        lcdNumber_x_3->setDigitCount(4);
        lcdNumber_x_3->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_x_3->setProperty("value", QVariant(0));
        lcdNumber_x_3->setProperty("intValue", QVariant(0));
        horizontalSlider_X_Camera = new QSlider(groupBox_2);
        horizontalSlider_X_Camera->setObjectName(QStringLiteral("horizontalSlider_X_Camera"));
        horizontalSlider_X_Camera->setGeometry(QRect(10, 50, 160, 29));
        horizontalSlider_X_Camera->setMinimum(-255);
        horizontalSlider_X_Camera->setMaximum(255);
        horizontalSlider_X_Camera->setValue(0);
        horizontalSlider_X_Camera->setSliderPosition(0);
        horizontalSlider_X_Camera->setOrientation(Qt::Horizontal);
        label_16 = new QLabel(groupBox_2);
        label_16->setObjectName(QStringLiteral("label_16"));
        label_16->setGeometry(QRect(10, 70, 191, 31));
        label_16->setFont(font1);
        lcdNumber_y_3 = new QLCDNumber(groupBox_2);
        lcdNumber_y_3->setObjectName(QStringLiteral("lcdNumber_y_3"));
        lcdNumber_y_3->setGeometry(QRect(180, 90, 61, 21));
        lcdNumber_y_3->setDigitCount(4);
        lcdNumber_y_3->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_y_3->setProperty("value", QVariant(0));
        lcdNumber_y_3->setProperty("intValue", QVariant(0));
        horizontalSlider_Y_Camera = new QSlider(groupBox_2);
        horizontalSlider_Y_Camera->setObjectName(QStringLiteral("horizontalSlider_Y_Camera"));
        horizontalSlider_Y_Camera->setGeometry(QRect(10, 90, 160, 29));
        horizontalSlider_Y_Camera->setMinimum(-255);
        horizontalSlider_Y_Camera->setMaximum(255);
        horizontalSlider_Y_Camera->setValue(0);
        horizontalSlider_Y_Camera->setOrientation(Qt::Horizontal);
        label_17 = new QLabel(groupBox_2);
        label_17->setObjectName(QStringLiteral("label_17"));
        label_17->setGeometry(QRect(10, 110, 191, 31));
        label_17->setFont(font1);
        horizontalSlider_Z_Camera = new QSlider(groupBox_2);
        horizontalSlider_Z_Camera->setObjectName(QStringLiteral("horizontalSlider_Z_Camera"));
        horizontalSlider_Z_Camera->setGeometry(QRect(10, 130, 160, 29));
        horizontalSlider_Z_Camera->setMinimum(-255);
        horizontalSlider_Z_Camera->setMaximum(255);
        horizontalSlider_Z_Camera->setValue(0);
        horizontalSlider_Z_Camera->setOrientation(Qt::Horizontal);
        lcdNumber_z_3 = new QLCDNumber(groupBox_2);
        lcdNumber_z_3->setObjectName(QStringLiteral("lcdNumber_z_3"));
        lcdNumber_z_3->setGeometry(QRect(180, 130, 61, 21));
        lcdNumber_z_3->setDigitCount(4);
        lcdNumber_z_3->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_z_3->setProperty("value", QVariant(0));
        lcdNumber_z_3->setProperty("intValue", QVariant(0));
        horizontalSlider_dx_Camera = new QSlider(groupBox_2);
        horizontalSlider_dx_Camera->setObjectName(QStringLiteral("horizontalSlider_dx_Camera"));
        horizontalSlider_dx_Camera->setGeometry(QRect(10, 170, 160, 29));
        horizontalSlider_dx_Camera->setMinimum(-180);
        horizontalSlider_dx_Camera->setMaximum(180);
        horizontalSlider_dx_Camera->setValue(0);
        horizontalSlider_dx_Camera->setOrientation(Qt::Horizontal);
        lcdNumber_dx_3 = new QLCDNumber(groupBox_2);
        lcdNumber_dx_3->setObjectName(QStringLiteral("lcdNumber_dx_3"));
        lcdNumber_dx_3->setGeometry(QRect(180, 170, 61, 21));
        lcdNumber_dx_3->setDigitCount(4);
        lcdNumber_dx_3->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_dx_3->setProperty("intValue", QVariant(0));
        label_18 = new QLabel(groupBox_2);
        label_18->setObjectName(QStringLiteral("label_18"));
        label_18->setGeometry(QRect(10, 150, 191, 31));
        label_18->setFont(font1);
        horizontalSlider_dy_Camera = new QSlider(groupBox_2);
        horizontalSlider_dy_Camera->setObjectName(QStringLiteral("horizontalSlider_dy_Camera"));
        horizontalSlider_dy_Camera->setGeometry(QRect(10, 210, 160, 29));
        horizontalSlider_dy_Camera->setMinimum(-180);
        horizontalSlider_dy_Camera->setMaximum(180);
        horizontalSlider_dy_Camera->setValue(0);
        horizontalSlider_dy_Camera->setOrientation(Qt::Horizontal);
        lcdNumber_dy_3 = new QLCDNumber(groupBox_2);
        lcdNumber_dy_3->setObjectName(QStringLiteral("lcdNumber_dy_3"));
        lcdNumber_dy_3->setGeometry(QRect(180, 210, 61, 21));
        lcdNumber_dy_3->setDigitCount(4);
        lcdNumber_dy_3->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_dy_3->setProperty("value", QVariant(0));
        lcdNumber_dy_3->setProperty("intValue", QVariant(0));
        label_19 = new QLabel(groupBox_2);
        label_19->setObjectName(QStringLiteral("label_19"));
        label_19->setGeometry(QRect(10, 190, 191, 31));
        label_19->setFont(font1);
        label_20 = new QLabel(groupBox_2);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(10, 230, 191, 31));
        label_20->setFont(font1);
        horizontalSlider_dz_Camera = new QSlider(groupBox_2);
        horizontalSlider_dz_Camera->setObjectName(QStringLiteral("horizontalSlider_dz_Camera"));
        horizontalSlider_dz_Camera->setGeometry(QRect(10, 250, 160, 29));
        horizontalSlider_dz_Camera->setMinimum(-180);
        horizontalSlider_dz_Camera->setMaximum(180);
        horizontalSlider_dz_Camera->setValue(0);
        horizontalSlider_dz_Camera->setOrientation(Qt::Horizontal);
        lcdNumber_dz_3 = new QLCDNumber(groupBox_2);
        lcdNumber_dz_3->setObjectName(QStringLiteral("lcdNumber_dz_3"));
        lcdNumber_dz_3->setGeometry(QRect(180, 250, 61, 21));
        lcdNumber_dz_3->setSmallDecimalPoint(false);
        lcdNumber_dz_3->setDigitCount(4);
        lcdNumber_dz_3->setMode(QLCDNumber::Dec);
        lcdNumber_dz_3->setSegmentStyle(QLCDNumber::Flat);
        lcdNumber_dz_3->setProperty("value", QVariant(0));
        lcdNumber_dz_3->setProperty("intValue", QVariant(0));
        textBrowser = new QTextBrowser(centralwidget);
        textBrowser->setObjectName(QStringLiteral("textBrowser"));
        textBrowser->setGeometry(QRect(1340, 61, 256, 461));
        textBrowser->setOverwriteMode(false);
        listView = new QListView(centralwidget);
        listView->setObjectName(QStringLiteral("listView"));
        listView->setGeometry(QRect(1340, 570, 256, 192));
        PCLViewer->setCentralWidget(centralwidget);

        retranslateUi(PCLViewer);
        QObject::connect(horizontalSlider_X_Object, SIGNAL(sliderMoved(int)), lcdNumber_x, SLOT(display(int)));
        QObject::connect(horizontalSlider_Y_Object, SIGNAL(sliderMoved(int)), lcdNumber_y, SLOT(display(int)));
        QObject::connect(horizontalSlider_Z_Object, SIGNAL(sliderMoved(int)), lcdNumber_z, SLOT(display(int)));
        QObject::connect(horizontalSlider_p, SIGNAL(sliderMoved(int)), lcdNumber_p, SLOT(display(int)));
        QObject::connect(horizontalSlider_dx_Object, SIGNAL(sliderMoved(int)), lcdNumber_dx, SLOT(display(int)));
        QObject::connect(horizontalSlider_dy_Object, SIGNAL(sliderMoved(int)), lcdNumber_dy, SLOT(display(int)));
        QObject::connect(horizontalSlider_dz_Object, SIGNAL(sliderMoved(int)), lcdNumber_dz, SLOT(display(int)));
        QObject::connect(horizontalSlider_X_Camera, SIGNAL(sliderMoved(int)), lcdNumber_x_3, SLOT(display(int)));
        QObject::connect(horizontalSlider_Y_Camera, SIGNAL(sliderMoved(int)), lcdNumber_y_3, SLOT(display(int)));
        QObject::connect(horizontalSlider_Z_Camera, SIGNAL(sliderMoved(int)), lcdNumber_z_3, SLOT(display(int)));
        QObject::connect(horizontalSlider_dx_Camera, SIGNAL(sliderMoved(int)), lcdNumber_dx_3, SLOT(display(int)));
        QObject::connect(horizontalSlider_dy_Camera, SIGNAL(sliderMoved(int)), lcdNumber_dy_3, SLOT(display(int)));
        QObject::connect(horizontalSlider_dz_Camera, SIGNAL(sliderMoved(int)), lcdNumber_dz_3, SLOT(display(int)));

        QMetaObject::connectSlotsByName(PCLViewer);
    } // setupUi

    void retranslateUi(QMainWindow *PCLViewer)
    {
        PCLViewer->setWindowTitle(QApplication::translate("PCLViewer", "PCLViewer", Q_NULLPTR));
        label_4->setText(QApplication::translate("PCLViewer", "Point size", Q_NULLPTR));
        groupBox->setTitle(QApplication::translate("PCLViewer", "Object_T_Base", Q_NULLPTR));
        label->setText(QApplication::translate("PCLViewer", "x", Q_NULLPTR));
        label_2->setText(QApplication::translate("PCLViewer", "y", Q_NULLPTR));
        label_3->setText(QApplication::translate("PCLViewer", "z", Q_NULLPTR));
        label_6->setText(QApplication::translate("PCLViewer", "rotx", Q_NULLPTR));
        label_7->setText(QApplication::translate("PCLViewer", "roty", Q_NULLPTR));
        label_8->setText(QApplication::translate("PCLViewer", "rotz", Q_NULLPTR));
        groupBox_2->setTitle(QApplication::translate("PCLViewer", "Camera_T_Flange", Q_NULLPTR));
        label_15->setText(QApplication::translate("PCLViewer", "x", Q_NULLPTR));
        label_16->setText(QApplication::translate("PCLViewer", "y", Q_NULLPTR));
        label_17->setText(QApplication::translate("PCLViewer", "z", Q_NULLPTR));
        label_18->setText(QApplication::translate("PCLViewer", "rotx", Q_NULLPTR));
        label_19->setText(QApplication::translate("PCLViewer", "roty", Q_NULLPTR));
        label_20->setText(QApplication::translate("PCLViewer", "rotz", Q_NULLPTR));
        textBrowser->setHtml(QApplication::translate("PCLViewer", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><br /></p></body></html>", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class PCLViewer: public Ui_PCLViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PCLVIEWER_H

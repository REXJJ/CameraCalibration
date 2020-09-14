/****************************************************************************
** Meta object code from reading C++ file 'pclviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.9.5)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../pclviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'pclviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.9.5. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
struct qt_meta_stringdata_PCLViewer_t {
    QByteArrayData data[18];
    char stringdata0[361];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_PCLViewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_PCLViewer_t qt_meta_stringdata_PCLViewer = {
    {
QT_MOC_LITERAL(0, 0, 9), // "PCLViewer"
QT_MOC_LITERAL(1, 10, 20), // "ObjectsliderReleased"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 20), // "CamerasliderReleased"
QT_MOC_LITERAL(4, 53, 19), // "pSliderValueChanged"
QT_MOC_LITERAL(5, 73, 5), // "value"
QT_MOC_LITERAL(6, 79, 19), // "xSliderValueChanged"
QT_MOC_LITERAL(7, 99, 19), // "ySliderValueChanged"
QT_MOC_LITERAL(8, 119, 19), // "zSliderValueChanged"
QT_MOC_LITERAL(9, 139, 20), // "dxSliderValueChanged"
QT_MOC_LITERAL(10, 160, 20), // "dySliderValueChanged"
QT_MOC_LITERAL(11, 181, 20), // "dzSliderValueChanged"
QT_MOC_LITERAL(12, 202, 25), // "xCameraSliderValueChanged"
QT_MOC_LITERAL(13, 228, 25), // "yCameraSliderValueChanged"
QT_MOC_LITERAL(14, 254, 25), // "zCameraSliderValueChanged"
QT_MOC_LITERAL(15, 280, 26), // "dxCameraSliderValueChanged"
QT_MOC_LITERAL(16, 307, 26), // "dyCameraSliderValueChanged"
QT_MOC_LITERAL(17, 334, 26) // "dzCameraSliderValueChanged"

    },
    "PCLViewer\0ObjectsliderReleased\0\0"
    "CamerasliderReleased\0pSliderValueChanged\0"
    "value\0xSliderValueChanged\0ySliderValueChanged\0"
    "zSliderValueChanged\0dxSliderValueChanged\0"
    "dySliderValueChanged\0dzSliderValueChanged\0"
    "xCameraSliderValueChanged\0"
    "yCameraSliderValueChanged\0"
    "zCameraSliderValueChanged\0"
    "dxCameraSliderValueChanged\0"
    "dyCameraSliderValueChanged\0"
    "dzCameraSliderValueChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_PCLViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      15,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   89,    2, 0x0a /* Public */,
       3,    0,   90,    2, 0x0a /* Public */,
       4,    1,   91,    2, 0x0a /* Public */,
       6,    1,   94,    2, 0x0a /* Public */,
       7,    1,   97,    2, 0x0a /* Public */,
       8,    1,  100,    2, 0x0a /* Public */,
       9,    1,  103,    2, 0x0a /* Public */,
      10,    1,  106,    2, 0x0a /* Public */,
      11,    1,  109,    2, 0x0a /* Public */,
      12,    1,  112,    2, 0x0a /* Public */,
      13,    1,  115,    2, 0x0a /* Public */,
      14,    1,  118,    2, 0x0a /* Public */,
      15,    1,  121,    2, 0x0a /* Public */,
      16,    1,  124,    2, 0x0a /* Public */,
      17,    1,  127,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,
    QMetaType::Void, QMetaType::Int,    5,

       0        // eod
};

void PCLViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        PCLViewer *_t = static_cast<PCLViewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->ObjectsliderReleased(); break;
        case 1: _t->CamerasliderReleased(); break;
        case 2: _t->pSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 3: _t->xSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: _t->ySliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 5: _t->zSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 6: _t->dxSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 7: _t->dySliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 8: _t->dzSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 9: _t->xCameraSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 10: _t->yCameraSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 11: _t->zCameraSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 12: _t->dxCameraSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 13: _t->dyCameraSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 14: _t->dzCameraSliderValueChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject PCLViewer::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_PCLViewer.data,
      qt_meta_data_PCLViewer,  qt_static_metacall, nullptr, nullptr}
};


const QMetaObject *PCLViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *PCLViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_meta_stringdata_PCLViewer.stringdata0))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int PCLViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 15)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 15;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 15)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 15;
    }
    return _id;
}
QT_WARNING_POP
QT_END_MOC_NAMESPACE

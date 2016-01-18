/****************************************************************************
** Meta object code from reading C++ file 'main_window.hpp'
**
** Created: Mon Feb 24 16:29:58 2014
**      by: The Qt Meta Object Compiler version 62 (Qt 4.6.2)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "main_window.hpp"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'main_window.hpp' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.6.2. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_pow_gui__MainWindow[] = {

 // content:
       4,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: signature, parameters, type, tag, flags
      21,   20,   20,   20, 0x0a,
      54,   48,   20,   20, 0x0a,
      86,   48,   20,   20, 0x0a,
     129,  123,   20,   20, 0x0a,
     178,  174,   20,   20, 0x0a,
     222,  174,   20,   20, 0x0a,
     265,   48,   20,   20, 0x0a,
     294,   20,   20,   20, 0x0a,
     312,   20,   20,   20, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_pow_gui__MainWindow[] = {
    "pow_gui::MainWindow\0\0on_actionAbout_triggered()\0"
    "check\0on_button_connect_clicked(bool)\0"
    "on_button_controlMotor_clicked(bool)\0"
    "state\0on_checkbox_useEnvironment_stateChanged(int)\0"
    "val\0on_doubleSpinBox_speed_valueChanged(double)\0"
    "on_doubleSpinBox_turn_valueChanged(double)\0"
    "on_button_stop_clicked(bool)\0"
    "onScoreReceived()\0updateLoggingView()\0"
};

const QMetaObject pow_gui::MainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_pow_gui__MainWindow,
      qt_meta_data_pow_gui__MainWindow, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &pow_gui::MainWindow::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *pow_gui::MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *pow_gui::MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_pow_gui__MainWindow))
        return static_cast<void*>(const_cast< MainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int pow_gui::MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: on_actionAbout_triggered(); break;
        case 1: on_button_connect_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: on_button_controlMotor_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: on_checkbox_useEnvironment_stateChanged((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 4: on_doubleSpinBox_speed_valueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 5: on_doubleSpinBox_turn_valueChanged((*reinterpret_cast< double(*)>(_a[1]))); break;
        case 6: on_button_stop_clicked((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 7: onScoreReceived(); break;
        case 8: updateLoggingView(); break;
        default: ;
        }
        _id -= 9;
    }
    return _id;
}
QT_END_MOC_NAMESPACE

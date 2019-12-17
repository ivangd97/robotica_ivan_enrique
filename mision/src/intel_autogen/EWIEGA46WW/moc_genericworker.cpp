/****************************************************************************
** Meta object code from reading C++ file 'genericworker.h'
**
** Created by: The Qt Meta Object Compiler version 63 (Qt 4.8.7)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../genericworker.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'genericworker.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 63
#error "This file was generated using the moc from 4.8.7. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_GenericWorker[] = {

 // content:
       6,       // revision
       0,       // classname
       0,    0, // classinfo
       9,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       4,       // signalCount

 // signals: signature, parameters, type, tag, flags
      15,   14,   14,   14, 0x05,
      22,   14,   14,   14, 0x05,
      48,   14,   14,   14, 0x05,
      71,   14,   14,   14, 0x05,

 // slots: signature, parameters, type, tag, flags
      95,   14,   14,   14, 0x0a,
     108,   14,   14,   14, 0x0a,
     124,   14,   14,   14, 0x0a,
     138,   14,   14,   14, 0x0a,
     155,  148,   14,   14, 0x0a,

       0        // eod
};

static const char qt_meta_stringdata_GenericWorker[] = {
    "GenericWorker\0\0kill()\0t_initialize_to_compute()\0"
    "t_compute_to_compute()\0t_compute_to_finalize()\0"
    "sm_compute()\0sm_initialize()\0sm_finalize()\0"
    "compute()\0period\0initialize(int)\0"
};

void GenericWorker::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        Q_ASSERT(staticMetaObject.cast(_o));
        GenericWorker *_t = static_cast<GenericWorker *>(_o);
        switch (_id) {
        case 0: _t->kill(); break;
        case 1: _t->t_initialize_to_compute(); break;
        case 2: _t->t_compute_to_compute(); break;
        case 3: _t->t_compute_to_finalize(); break;
        case 4: _t->sm_compute(); break;
        case 5: _t->sm_initialize(); break;
        case 6: _t->sm_finalize(); break;
        case 7: _t->compute(); break;
        case 8: _t->initialize((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObjectExtraData GenericWorker::staticMetaObjectExtraData = {
    0,  qt_static_metacall 
};

const QMetaObject GenericWorker::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_GenericWorker,
      qt_meta_data_GenericWorker, &staticMetaObjectExtraData }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &GenericWorker::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *GenericWorker::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *GenericWorker::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_GenericWorker))
        return static_cast<void*>(const_cast< GenericWorker*>(this));
    return QObject::qt_metacast(_clname);
}

int GenericWorker::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 9)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 9;
    }
    return _id;
}

// SIGNAL 0
void GenericWorker::kill()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}

// SIGNAL 1
void GenericWorker::t_initialize_to_compute()
{
    QMetaObject::activate(this, &staticMetaObject, 1, 0);
}

// SIGNAL 2
void GenericWorker::t_compute_to_compute()
{
    QMetaObject::activate(this, &staticMetaObject, 2, 0);
}

// SIGNAL 3
void GenericWorker::t_compute_to_finalize()
{
    QMetaObject::activate(this, &staticMetaObject, 3, 0);
}
QT_END_MOC_NAMESPACE

#ifndef KALMANPOSITIONATTRCONTROLLER_H
#define KALMANPOSITIONATTRCONTROLLER_H

#include "core/IKalmanPositionAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

class KalmanPositionAttrController : public QObject, AttrControllerBase<IKalmanPositionAttr>
{
    Q_OBJECT

public:
    KalmanPositionAttrController(QLineEdit * proc_accel_std_le,
                                 QLineEdit * meas_pos_std_le, QLineEdit * meas_vel_std_le,
                                 QLineEdit * init_pos_std_le, QLineEdit * init_vel_std_le, QLineEdit * init_accel_std_le);

    using AttrControllerBase<IKalmanPositionAttr>::set_model;
    void apply_attributes();
    void borrow_attributes();

public slots:
    void on_proc_accel_std(const QString & str);
    void on_meas_pos_std(const QString & str);
    void on_meas_vel_std(const QString & str);
    void on_init_pos_std(const QString & str);
    void on_init_vel_std(const QString & str);
    void on_init_accel_std(const QString & str);

private:
    QLineEdit * proc_accel_std_le;
    QLineEdit * meas_pos_std_le;
    QLineEdit * meas_vel_std_le;
    QLineEdit * init_pos_std_le;
    QLineEdit * init_vel_std_le;
    QLineEdit * init_accel_std_le;
};

#endif // KALMANPOSITIONFILTERINGCONTROLLER_H

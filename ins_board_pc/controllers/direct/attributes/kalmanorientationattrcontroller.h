#ifndef KALMANORIENTATIONATTRCONTROLLER_H
#define KALMANORIENTATIONATTRCONTROLLER_H

#include "core/IKalmanOrientationAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

class KalmanOrientationAttrController : public QObject, AttrControllerBase<IKalmanOrientationAttr>
{
    Q_OBJECT

public:
    KalmanOrientationAttrController(QLineEdit * proc_gyro_le, QLineEdit * proc_gyro_bias_le,
                                    QLineEdit * meas_accel_std_le, QLineEdit * meas_magn_std_le,
                                    QLineEdit * init_qs_std_le, QLineEdit * init_qx_std_le, QLineEdit * init_qy_std_le, QLineEdit * init_qz_std_le,
                                    QLineEdit * init_bias_std_le);

    using::AttrControllerBase<IKalmanOrientationAttr>::set_model;
    void apply_attributes();

public slots:
    void on_proc_gyro_std(const QString & str);
    void on_proc_gyro_bias_std(const QString & str);
    void on_meas_accel_std(const QString & str);
    void on_meas_magn_std(const QString & str);
    void on_init_qs_std(const QString & str);
    void on_init_qx_std(const QString & str);
    void on_init_qy_std(const QString & str);
    void on_init_qz_std(const QString & str);
    void on_init_bias_std(const QString & str);

private:
    QLineEdit * proc_gyro_le;
    QLineEdit * proc_gyro_bias_le;
    QLineEdit * meas_accel_std_le;
    QLineEdit * meas_magn_std_le;
    QLineEdit * init_qs_std_le;
    QLineEdit * init_qx_std_le;
    QLineEdit * init_qy_std_le;
    QLineEdit * init_qz_std_le;
    QLineEdit * init_bias_std_le;
};

#endif // KALMANORIENTATIONFILTERINGCONTROLLER_H

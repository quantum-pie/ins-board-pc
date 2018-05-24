#include "controllers/kalmanpositionattrcontroller.h"
#include <QLineEdit>

KalmanPositionAttrController::KalmanPositionAttrController(QLineEdit *proc_accel_std_le,
                                                           QLineEdit *meas_pos_std_le, QLineEdit *meas_vel_std_le,
                                                           QLineEdit *init_pos_std_le, QLineEdit *init_vel_std_le, QLineEdit *init_accel_std_le)
    : proc_accel_std_le{ proc_accel_std_le },
      meas_pos_std_le{ meas_pos_std_le }, meas_vel_std_le{ meas_vel_std_le },
      init_pos_std_le{ init_pos_std_le }, init_vel_std_le{ init_vel_std_le }, init_accel_std_le{ init_accel_std_le }
{
    connect(proc_accel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_proc_accel_std(QString)));
    connect(meas_pos_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_meas_pos_std(QString)));
    connect(meas_vel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_meas_vel_std(QString)));
    connect(init_pos_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_init_pos_std(QString)));
    connect(init_accel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_init_accel_std(QString)));
    connect(init_vel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_init_vel_std(QString)));
}


void KalmanPositionAttrController::on_proc_accel_std(const QString & str)
{
    call_setter(str, &IKalmanPositionAttr::set_proc_accel_std);
}

void KalmanPositionAttrController::on_meas_pos_std(const QString & str)
{
    call_setter(str, &IKalmanPositionAttr::set_meas_pos_std);
}

void KalmanPositionAttrController::on_meas_vel_std(const QString & str)
{
    call_setter(str, &IKalmanPositionAttr::set_meas_vel_std);
}

void KalmanPositionAttrController::on_init_pos_std(const QString & str)
{
    call_setter(str, &IKalmanPositionAttr::set_init_pos_std);
}

void KalmanPositionAttrController::on_init_vel_std(const QString & str)
{
    call_setter(str, &IKalmanPositionAttr::set_init_vel_std);
}

void KalmanPositionAttrController::on_init_accel_std(const QString & str)
{
    call_setter(str, &IKalmanPositionAttr::set_init_accel_std);
}

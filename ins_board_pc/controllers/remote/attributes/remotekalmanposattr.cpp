#include "controllers/remote/attributes/remotekalmanposattr.h"

#include <QLineEdit>
#include <QPushButton>

RemoteKalmanPosAttr::RemoteKalmanPosAttr(TerminalBase & tbase, QPushButton * save_btn, QLineEdit *proc_accel_std_le,
                                                           QLineEdit *meas_pos_std_le, QLineEdit *meas_vel_std_le,
                                                           QLineEdit *init_pos_std_le, QLineEdit *init_vel_std_le, QLineEdit *init_accel_std_le)
    : var{ tbase }, proc_accel_std_le{ proc_accel_std_le },
      meas_pos_std_le{ meas_pos_std_le }, meas_vel_std_le{ meas_vel_std_le },
      init_pos_std_le{ init_pos_std_le }, init_vel_std_le{ init_vel_std_le }, init_accel_std_le{ init_accel_std_le }
{
    connect(save_btn, SIGNAL(released()), this, SLOT(on_save_bnt()));
    connect(proc_accel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_proc_accel_std(QString)));
    connect(meas_pos_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_meas_pos_std(QString)));
    connect(meas_vel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_meas_vel_std(QString)));
    connect(init_pos_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_init_pos_std(QString)));
    connect(init_accel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_init_accel_std(QString)));
    connect(init_vel_std_le, SIGNAL(textEdited(QString)), this, SLOT(on_init_vel_std(QString)));

    borrow_attributes();
}

void RemoteKalmanPosAttr::on_save_bnt()
{
    var.set("save", "");
}

void RemoteKalmanPosAttr::on_proc_accel_std(const QString & str)
{
    var.set("kalman.proc.a", str.toStdString());
}

void RemoteKalmanPosAttr::on_meas_pos_std(const QString & str)
{
    var.set("kalman.meas.p", str.toStdString());
}

void RemoteKalmanPosAttr::on_meas_vel_std(const QString & str)
{
    var.set("kalman.meas.v", str.toStdString());
}

void RemoteKalmanPosAttr::on_init_pos_std(const QString & str)
{
    var.set("kalman.init.p", str.toStdString());
}

void RemoteKalmanPosAttr::on_init_vel_std(const QString & str)
{
    var.set("kalman.init.v", str.toStdString());
}

void RemoteKalmanPosAttr::on_init_accel_std(const QString & str)
{
    var.set("kalman.init.a", str.toStdString());
}

void RemoteKalmanPosAttr::borrow_attributes()
{
    init_accel_std_le->setText(QString::number(var.get<double>("kalman.init.a")));
    init_pos_std_le->setText(QString::number(var.get<double>("kalman.init.p")));
    init_vel_std_le->setText(QString::number(var.get<double>("kalman.init.v")));
    meas_pos_std_le->setText(QString::number(var.get<double>("kalman.meas.p")));
    meas_vel_std_le->setText(QString::number(var.get<double>("kalman.meas.v")));
    proc_accel_std_le->setText(QString::number(var.get<double>("kalman.proc.a")));
}

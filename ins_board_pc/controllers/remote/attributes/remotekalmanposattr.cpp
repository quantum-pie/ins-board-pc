#include "controllers/remote/attributes/remotekalmanposattr.h"
#include "communication/terminalbase.h"

#include <QLineEdit>

RemoteKalmanPosAttr::RemoteKalmanPosAttr(TerminalBase & tbase, QLineEdit *proc_accel_std_le,
                                                           QLineEdit *meas_pos_std_le, QLineEdit *meas_vel_std_le,
                                                           QLineEdit *init_pos_std_le, QLineEdit *init_vel_std_le, QLineEdit *init_accel_std_le)
    : tbase{ tbase }, proc_accel_std_le{ proc_accel_std_le },
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

void RemoteKalmanPosAttr::on_proc_accel_std(const QString & str)
{
    tbase.send_text("kalman.proc.a " + str.toStdString() + "\n");
}

void RemoteKalmanPosAttr::on_meas_pos_std(const QString & str)
{
    tbase.send_text("kalman.meas.p " + str.toStdString() + "\n");
}

void RemoteKalmanPosAttr::on_meas_vel_std(const QString & str)
{
    tbase.send_text("kalman.meas.v " + str.toStdString() + "\n");
}

void RemoteKalmanPosAttr::on_init_pos_std(const QString & str)
{
    tbase.send_text("kalman.init.p " + str.toStdString() + "\n");
}

void RemoteKalmanPosAttr::on_init_vel_std(const QString & str)
{
    tbase.send_text("kalman.init.v " + str.toStdString() + "\n");
}

void RemoteKalmanPosAttr::on_init_accel_std(const QString & str)
{
    tbase.send_text("kalman.init.a " + str.toStdString() + "\n");
}

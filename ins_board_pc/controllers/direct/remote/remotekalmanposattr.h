#ifndef REMOTEKALMANPOSATTR_H
#define REMOTEKALMANPOSATTR_H

#include <QObject>

class QLineEdit;
class TerminalBase;

class RemoteKalmanPosAttr : public QObject
{
    Q_OBJECT

public:
    RemoteKalmanPosAttr(TerminalBase & tbase,
                        QLineEdit * proc_accel_std_le, QLineEdit * meas_pos_std_le, QLineEdit * meas_vel_std_le,
                        QLineEdit * init_pos_std_le, QLineEdit * init_vel_std_le, QLineEdit * init_accel_std_le);

public slots:
    void on_proc_accel_std(const QString & str);
    void on_meas_pos_std(const QString & str);
    void on_meas_vel_std(const QString & str);
    void on_init_pos_std(const QString & str);
    void on_init_vel_std(const QString & str);
    void on_init_accel_std(const QString & str);

private:
    TerminalBase & tbase;
    QLineEdit * proc_accel_std_le;
    QLineEdit * meas_pos_std_le;
    QLineEdit * meas_vel_std_le;
    QLineEdit * init_pos_std_le;
    QLineEdit * init_vel_std_le;
    QLineEdit * init_accel_std_le;
};

#endif // REMOTEKALMANPOSATTR_H

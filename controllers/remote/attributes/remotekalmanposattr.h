#ifndef REMOTEKALMANPOSATTR_H
#define REMOTEKALMANPOSATTR_H

#include "communication/var.h"

#include <QObject>

class QLineEdit;
class TerminalBase;
class QPushButton;

/*!
 * @brief The RemoteKalmanPosAttr class
 * This is the controller of remote Kalman position filter attributes.
 */
class RemoteKalmanPosAttr : public QObject
{
    Q_OBJECT

public:
    /*!
     * @brief RemoteKalmanPosAttr constructor.
     * @param tbase TerminalBase reference.
     * @param save_btn Save widget.
     * @param proc_accel_std_le Acceleration process noise std LineEdit.
     * @param meas_pos_std_le Position measurement noise std LineEdit.
     * @param meas_vel_std_le Velocity measurement noise std LineEdit.
     * @param init_pos_std_le Initial position estimate std LineEdit.
     * @param init_vel_std_le Initial velocity estimate std LineEdit.
     * @param init_accel_std_le Initial acceleration estimate std LineEdit.
     */
    RemoteKalmanPosAttr(TerminalBase & tbase, QPushButton * save_btn,
                        QLineEdit * proc_accel_std_le, QLineEdit * meas_pos_std_le, QLineEdit * meas_vel_std_le,
                        QLineEdit * init_pos_std_le, QLineEdit * init_vel_std_le, QLineEdit * init_accel_std_le);

    /*!
     * @brief Borrow attributes from the remote filter.
     */
    void borrow_attributes();

public slots:
    /*!
     * @brief Save remote attributes.
     */
    void on_save_bnt();

    /*!
     * @brief Acceleration process noise std changed.
     * @param str new value string.
     */
    void on_proc_accel_std(const QString & str);

    /*!
     * @brief Position measurement noise std changed.
     * @param str new value string.
     */
    void on_meas_pos_std(const QString & str);

    /*!
     * @brief Velocity measurement noise std changed.
     * @param str new value string.
     */
    void on_meas_vel_std(const QString & str);

    /*!
     * @brief Initial position estimate std changed.
     * @param str new value string.
     */
    void on_init_pos_std(const QString & str);

    /*!
     * @brief Initial velocity estimate std changed.
     * @param str new value string.
     */
    void on_init_vel_std(const QString & str);

    /*!
     * @brief Initial acceleration estimate std changed.
     * @param str new value string.
     */
    void on_init_accel_std(const QString & str);

private:
    Var var;
    QLineEdit * proc_accel_std_le;
    QLineEdit * meas_pos_std_le;
    QLineEdit * meas_vel_std_le;
    QLineEdit * init_pos_std_le;
    QLineEdit * init_vel_std_le;
    QLineEdit * init_accel_std_le;
};

#endif // REMOTEKALMANPOSATTR_H

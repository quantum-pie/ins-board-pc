#ifndef KALMANORIENTATIONATTRCONTROLLER_H
#define KALMANORIENTATIONATTRCONTROLLER_H

#include "core/IKalmanOrientationAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

/*!
 * @brief The KalmanOrientationAttrController class
 * This is the controller of Kalman orientation filter attributes.
 */
class KalmanOrientationAttrController : public QObject, AttrControllerBase<IKalmanOrientationAttr>
{
    Q_OBJECT

public:
    /*!
     * @brief KalmanOrientationAttrController constructor.
     * @param proc_gyro_le Angular rate process noise std LineEdit.
     * @param proc_gyro_bias_le Gyroscope bias process noise std LineEdit.
     * @param meas_accel_std_le Accelerometer measurement noise std LineEdit.
     * @param meas_magn_std_le Magnetometer measurement noise std LineEdit.
     * @param init_qs_std_le Initial qs estimate std LineEdit.
     * @param init_qx_std_le Initial qx estimate std LineEdit.
     * @param init_qy_std_le Initial qy estimate std LineEdit.
     * @param init_qz_std_le Initial qz estimate std LineEdit.
     * @param init_bias_std_le Initial bias estimate std LineEdit.
     */
    KalmanOrientationAttrController(QLineEdit * proc_gyro_le, QLineEdit * proc_gyro_bias_le,
                                    QLineEdit * meas_accel_std_le, QLineEdit * meas_magn_std_le,
                                    QLineEdit * init_qs_std_le, QLineEdit * init_qx_std_le, QLineEdit * init_qy_std_le, QLineEdit * init_qz_std_le,
                                    QLineEdit * init_bias_std_le);

    //! Bring model setter to scope.
    using::AttrControllerBase<IKalmanOrientationAttr>::set_model;

    /*!
     * @brief Apply attributes to the current model.
     */
    void apply_attributes();

    /*!
     * @brief Borrow attributes from the current model.
     */
    void borrow_attributes();

public slots:
    /*!
     * @brief Angular rate process noise std changed.
     * @param str new value string.
     */
    void on_proc_gyro_std(const QString & str);

    /*!
     * @brief Gyroscope bias process noise std changed.
     * @param str new value string.
     */
    void on_proc_gyro_bias_std(const QString & str);

    /*!
     * @brief Accelerometer measurement noise std changed.
     * @param str new value string.
     */

    void on_meas_accel_std(const QString & str);

    /*!
     * @brief Magnetometer measurement noise std changed.
     * @param str new value string.
     */
    void on_meas_magn_std(const QString & str);

    /*!
     * @brief Initial qs estimate std changed.
     * @param str new value string.
     */
    void on_init_qs_std(const QString & str);

    /*!
     * @brief Initial qx estimate std changed.
     * @param str new value string.
     */
    void on_init_qx_std(const QString & str);

    /*!
     * @brief Initial qy estimate std changed.
     * @param str new value string.
     */
    void on_init_qy_std(const QString & str);

    /*!
     * @brief Initial qz estimate std changed.
     * @param str new value string.
     */
    void on_init_qz_std(const QString & str);

    /*!
     * @brief Initial bias estimate std changed.
     * @param str new value string.
     */
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

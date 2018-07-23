#ifndef KALMANPOSITIONATTRCONTROLLER_H
#define KALMANPOSITIONATTRCONTROLLER_H

#include "core/IKalmanPositionAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

/*!
 * @brief The KalmanPositionAttrController class
 * This is the controller of Kalman position filter attributes.
 */
class KalmanPositionAttrController : public QObject, AttrControllerBase<IKalmanPositionAttr>
{
    Q_OBJECT

public:
    /*!
     * @brief KalmanPositionAttrController constructor.
     * @param proc_accel_std_le Acceleration process noise std LineEdit.
     * @param meas_pos_std_le Position measurement noise std LineEdit.
     * @param meas_vel_std_le Velocity measurement noise std LineEdit.
     * @param init_pos_std_le Initial position estimate std LineEdit.
     * @param init_vel_std_le Initial velocity estimate std LineEdit.
     * @param init_accel_std_le Initial acceleration estimate std LineEdit.
     */
    KalmanPositionAttrController(QLineEdit * proc_accel_std_le,
                                 QLineEdit * meas_pos_std_le, QLineEdit * meas_vel_std_le,
                                 QLineEdit * init_pos_std_le, QLineEdit * init_vel_std_le, QLineEdit * init_accel_std_le);

    //! Bring model setter to scope.
    using AttrControllerBase<IKalmanPositionAttr>::set_model;

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
    QLineEdit * proc_accel_std_le;
    QLineEdit * meas_pos_std_le;
    QLineEdit * meas_vel_std_le;
    QLineEdit * init_pos_std_le;
    QLineEdit * init_vel_std_le;
    QLineEdit * init_accel_std_le;
};

#endif // KALMANPOSITIONFILTERINGCONTROLLER_H

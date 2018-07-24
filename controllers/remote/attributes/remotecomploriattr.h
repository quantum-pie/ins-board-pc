#ifndef REMOTECOMPLORIATTR_H
#define REMOTECOMPLORIATTR_H

#include "communication/var.h"

#include <QObject>

class QLineEdit;
class TerminalBase;
class QPushButton;

/*!
 * @brief The RemoteComplOriAttr struct
 * This is the controller of remote complementary orientation filter attributes.
 */
struct RemoteComplOriAttr : public QObject
{
    Q_OBJECT

public:
    /*!
     * @brief RemoteComplOriAttr constructor.
     * @param tbase TerminalBase reference.
     * @param save_btn Save widget.
     * @param static_accel_le Static accelerometer gain LineEdit.
     * @param static_magn_le Static magnetometer gain LineEdit.
     * @param bias_gain_le Bias gain LineEdit.
     */
    RemoteComplOriAttr(TerminalBase & tbase, QPushButton * save_btn, QLineEdit * static_accel_le, QLineEdit * static_magn_le, QLineEdit * bias_gain_le);

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
     * @brief Static accelerometer gain changed.
     * @param str new gain string.
     */
    void on_static_accel_gain(const QString & str);

    /*!
     * @brief Static magnetometer gain changed.
     * @param str new gain string.
     */
    void on_static_magn_gain(const QString & str);

    /*!
     * @brief Bias gain changed.
     * @param str new gain string.
     */
    void on_bias_gain(const QString & str);

private:
    Var var;
    QLineEdit * static_accel_le;
    QLineEdit * static_magn_le;
    QLineEdit * bias_gain_le;
};

#endif // REMOTECOMPLORIATTR_H

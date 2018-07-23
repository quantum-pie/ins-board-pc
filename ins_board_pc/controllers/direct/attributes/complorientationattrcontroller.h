#ifndef COMPLORIENTATIONATTRCONTROLLER_H
#define COMPLORIENTATIONATTRCONTROLLER_H

#include "core/IComplementOrientationAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

/*!
 * @brief The ComplOrientationAttrController class
 * This is the controller of complementary orientation filter attributes.
 */
class ComplOrientationAttrController : public QObject, AttrControllerBase<IComplementOrientationAttr>
{
    Q_OBJECT

public:
    /*!
     * @brief ComplOrientationAttrController constructor.
     * @param static_accel_le Static accelerometer gain LineEdit.
     * @param static_magn_le Static magnetometer gain LineEdit.
     * @param bias_gain_le Bias gain LineEdit.
     */
    ComplOrientationAttrController(QLineEdit * static_accel_le, QLineEdit * static_magn_le, QLineEdit * bias_gain_le);

    //! Bring model setter to scope.
    using AttrControllerBase<IComplementOrientationAttr>::set_model;

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
    QLineEdit * static_accel_le;
    QLineEdit * static_magn_le;
    QLineEdit * bias_gain_le;
};

#endif // COMPLORIENTATIONATTRCONTROLLER_H

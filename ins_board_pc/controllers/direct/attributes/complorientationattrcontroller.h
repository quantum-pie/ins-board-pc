#ifndef COMPLORIENTATIONATTRCONTROLLER_H
#define COMPLORIENTATIONATTRCONTROLLER_H

#include "core/IComplementOrientationAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

class ComplOrientationAttrController : public QObject, public AttrControllerBase<IComplementOrientationAttr>
{
    Q_OBJECT

public:
    ComplOrientationAttrController(QLineEdit * static_accel_le, QLineEdit * static_magn_le, QLineEdit * bias_gain_le);


public slots:
    void on_static_accel_gain(const QString & str);
    void on_static_magn_gain(const QString & str);
    void on_bias_gain(const QString & str);

private:
    QLineEdit * static_accel_le;
    QLineEdit * static_magn_le;
    QLineEdit * bias_gain_le;
};

#endif // COMPLORIENTATIONATTRCONTROLLER_H

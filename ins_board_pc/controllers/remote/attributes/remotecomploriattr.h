#ifndef REMOTECOMPLORIATTR_H
#define REMOTECOMPLORIATTR_H

#include "communication/var.h"

#include <QObject>

class QLineEdit;
class TerminalBase;

struct RemoteComplOriAttr : public QObject
{
    Q_OBJECT

public:
    RemoteComplOriAttr(TerminalBase & tbase, QLineEdit * static_accel_le, QLineEdit * static_magn_le, QLineEdit * bias_gain_le);
    void borrow_attributes();

public slots:
    void on_static_accel_gain(const QString & str);
    void on_static_magn_gain(const QString & str);
    void on_bias_gain(const QString & str);

private:
    Var var;
    QLineEdit * static_accel_le;
    QLineEdit * static_magn_le;
    QLineEdit * bias_gain_le;
};

#endif // REMOTECOMPLORIATTR_H

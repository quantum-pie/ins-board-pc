#include "controllers/remote/attributes/remotecomploriattr.h"
#include "communication/terminalbase.h"

#include <QLineEdit>

RemoteComplOriAttr::RemoteComplOriAttr(TerminalBase &tbase, QLineEdit *static_accel_le, QLineEdit *static_magn_le, QLineEdit *bias_gain_le)
    : tbase{ tbase }, static_accel_le{ static_accel_le }, static_magn_le{ static_magn_le }, bias_gain_le{ bias_gain_le }
{
    connect(static_accel_le, SIGNAL(textEdited(QString)), this, SLOT(on_static_accel_gain(QString)));
    connect(static_magn_le, SIGNAL(textEdited(QString)), this, SLOT(on_static_magn_gain(QString)));
    connect(bias_gain_le, SIGNAL(textEdited(QString)), this, SLOT(on_bias_gain(QString)));
}

void RemoteComplOriAttr::on_static_accel_gain(const QString &str)
{
    tbase.send_text("madg.a " + str.toStdString() + "\n");
}

void RemoteComplOriAttr::on_static_magn_gain(const QString &str)
{
    tbase.send_text("madg.m " + str.toStdString() + "\n");
}

void RemoteComplOriAttr::on_bias_gain(const QString &str)
{
    tbase.send_text("madg.b " + str.toStdString() + "\n");
}

void RemoteComplOriAttr::borrow_attributes()
{
    //TODO
}

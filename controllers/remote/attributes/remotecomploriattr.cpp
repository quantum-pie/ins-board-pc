#include "controllers/remote/attributes/remotecomploriattr.h"

#include <QLineEdit>
#include <QPushButton>

RemoteComplOriAttr::RemoteComplOriAttr(TerminalBase & tbase, QPushButton * save_btn, QLineEdit *static_accel_le, QLineEdit *static_magn_le, QLineEdit *bias_gain_le)
    : var{ tbase }, static_accel_le{ static_accel_le }, static_magn_le{ static_magn_le }, bias_gain_le{ bias_gain_le }
{
    connect(save_btn, SIGNAL(released()), this, SLOT(on_save_bnt()));
    connect(static_accel_le, SIGNAL(textEdited(QString)), this, SLOT(on_static_accel_gain(QString)));
    connect(static_magn_le, SIGNAL(textEdited(QString)), this, SLOT(on_static_magn_gain(QString)));
    connect(bias_gain_le, SIGNAL(textEdited(QString)), this, SLOT(on_bias_gain(QString)));

    borrow_attributes();
}

void RemoteComplOriAttr::on_save_bnt()
{
    var.set("save", "");
}

void RemoteComplOriAttr::on_static_accel_gain(const QString &str)
{
    var.set("madg.a", str.toStdString());
}

void RemoteComplOriAttr::on_static_magn_gain(const QString &str)
{
    var.set("madg.m", str.toStdString());
}

void RemoteComplOriAttr::on_bias_gain(const QString &str)
{
    var.set("madg.b", str.toStdString());
}

void RemoteComplOriAttr::borrow_attributes()
{
    static_accel_le->setText(QString::number(var.get<double>("madg.a")));
    static_magn_le->setText(QString::number(var.get<double>("madg.m")));
    bias_gain_le->setText(QString::number(var.get<double>("madg.b")));
}

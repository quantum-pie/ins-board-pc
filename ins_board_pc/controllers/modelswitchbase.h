#ifndef BASEMODELSWITCH_H
#define BASEMODELSWITCH_H

#include <QObject>
#include <QComboBox>

class ModelSwitchBase : public QObject
{
    Q_OBJECT

public:
    ModelSwitchBase(QComboBox * sw) : sw{ sw } {}

    void enable() { sw->setEnabled(true); }
    void disable() { sw->setEnabled(false); }

public slots:
    void switch_model(int cb_idx) {}

private:
    QComboBox * sw;
};

#endif // BASEMODELSWITCH_H

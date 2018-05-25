#ifndef BASEMODELSWITCH_H
#define BASEMODELSWITCH_H

#include <QObject>

class QComboBox;

class ModelSwitchBase : public QObject
{
    Q_OBJECT

public:
    ModelSwitchBase(QComboBox * sw);

    void enable();
    void disable();

public slots:
    void switch_model(int) {}

private:
    QComboBox * sw;
};

#endif // BASEMODELSWITCH_H

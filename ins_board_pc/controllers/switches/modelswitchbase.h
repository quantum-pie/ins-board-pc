#ifndef BASEMODELSWITCH_H
#define BASEMODELSWITCH_H

#include <QObject>

class QComboBox;

class ModelSwitchBase : public QObject
{
    Q_OBJECT

public:
    ModelSwitchBase(QComboBox * sw);
    virtual ~ModelSwitchBase() = default;

    void enable();
    void disable();

public slots:
    virtual void switch_model(int) = 0;

private:
    QComboBox * sw;
};

#endif // BASEMODELSWITCH_H

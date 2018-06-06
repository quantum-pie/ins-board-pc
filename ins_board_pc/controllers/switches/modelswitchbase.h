#ifndef BASEMODELSWITCH_H
#define BASEMODELSWITCH_H

#include <QObject>

#include <QDebug>

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
    virtual void switch_model(int)
    {
        qDebug() << "Swithc model\n";
    }

private:
    QComboBox * sw;
};

#endif // BASEMODELSWITCH_H

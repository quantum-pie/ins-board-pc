#ifndef SIMPOSITIONATTRCONTROLLER_H
#define SIMPOSITIONATTRCONTROLLER_H

#include "core/ISimPositionAttr.h"
#include "controllers/direct/attributes/attrcontrollerbase.h"

#include <QObject>

class QLineEdit;

class SimPositionAttrController : public QObject, AttrControllerBase<ISimPositionAttr>
{
    Q_OBJECT

public:
    SimPositionAttrController(QLineEdit * speed_le, QLineEdit * initial_track_le);

    using AttrControllerBase<ISimPositionAttr>::set_model;
    void apply_attributes();
    void borrow_attributes();

public slots:
    void on_speed(const QString & str);
    void on_initial_track(const QString & str);

private:
    QLineEdit * speed_le;
    QLineEdit * initial_track_le;
};

#endif // SIMPOSITIONATTRCONTROLLER_H

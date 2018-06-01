#include "controllers/direct/attributes/simpositionattrcontroller.h"
#include <QLineEdit>

SimPositionAttrController::SimPositionAttrController(QLineEdit *speed_le, QLineEdit *initial_track_le)
    : speed_le{ speed_le }, initial_track_le{ initial_track_le }
{
    connect(speed_le, SIGNAL(textEdited(QString)), this, SLOT(on_speed(QString)));
    connect(initial_track_le, SIGNAL(textEdited(QString)), this, SLOT(on_initial_track(QString)));
}

void SimPositionAttrController::on_speed(const QString &str)
{
    call_setter(str, &ISimPositionAttr::set_speed);
}

void SimPositionAttrController::on_initial_track(const QString &str)
{
    call_setter(str, &ISimPositionAttr::set_initial_track);
}

void SimPositionAttrController::apply_attributes()
{
    on_initial_track(initial_track_le->text());
    on_speed(speed_le->text());
}

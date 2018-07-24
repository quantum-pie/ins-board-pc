#include "controllers/switches/modelswitchbase.h"

#include <QComboBox>

ModelSwitchBase::ModelSwitchBase(QComboBox *sw) : sw{ sw } {}

void ModelSwitchBase::enable()
{
    sw->setEnabled(true);
    emit sw->activated(sw->currentIndex());
}

void ModelSwitchBase::disable()
{
    sw->setEnabled(false);
}



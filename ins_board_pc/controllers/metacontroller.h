#ifndef METACONTROLLER_H
#define METACONTROLLER_H

#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/switches/orientationmodelswitch.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/mixedmodelswitch.h"

#include <QObject>
#include <memory>

class QComboBox;

class MetaController : public QObject
{
    Q_OBJECT

public:
    MetaController(QComboBox * meta_cb, std::shared_ptr<PositionModelSwitch> pos_sw, std::shared_ptr<OrientationModelSwitch> ori_sw,
                   std::unique_ptr<MixedModelSwitch> mix_sw, std::shared_ptr<PositionFilteringController> pos_ctrl);

public slots:
    void configure_control(int cb_idx);

private:
    std::shared_ptr<PositionModelSwitch> pos_sw;
    std::shared_ptr<OrientationModelSwitch> ori_sw;
    std::unique_ptr<MixedModelSwitch> mix_sw;
    std::shared_ptr<PositionFilteringController> pos_ctrl;
};

#endif // METACONTROLLER_H

#ifndef METACONTROLLER_H
#define METACONTROLLER_H

#include "controllers/direct/filtering/filteringcontrollersfwd.h"
#include "controllers/switches/orientationmodelswitch.h"
#include "controllers/switches/positionmodelswitch.h"
#include "controllers/switches/mixedmodelswitch.h"

#include <QObject>
#include <memory>

class QComboBox;

/*!
 * @brief The MetaController class
 * This class is designed to switch between single and mixed switches.
 */
class MetaController : public QObject
{
    Q_OBJECT

public:
    /*!
     * @brief MetaController constructor.
     * @param meta_cb Switch widget.
     * @param pos_sw Position model switch.
     * @param ori_sw Orientation model switch.
     * @param mix_sw Mixed model switch.
     * @param pos_ctrl Position filtering controller.
     */
    MetaController(QComboBox * meta_cb, std::shared_ptr<PositionModelSwitch> pos_sw, std::shared_ptr<OrientationModelSwitch> ori_sw,
                   std::unique_ptr<MixedModelSwitch> mix_sw, std::shared_ptr<PositionFilteringController> pos_ctrl);

public slots:
    /*!
     * @brief Set switches configuration.
     * @param cb_idx switches configuration index.
     */
    void configure_control(int cb_idx);

private:
    std::shared_ptr<PositionModelSwitch> pos_sw;
    std::shared_ptr<OrientationModelSwitch> ori_sw;
    std::unique_ptr<MixedModelSwitch> mix_sw;
    std::shared_ptr<PositionFilteringController> pos_ctrl;
};

#endif // METACONTROLLER_H

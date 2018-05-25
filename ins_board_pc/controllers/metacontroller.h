#ifndef METACONTROLLER_H
#define METACONTROLLER_H

#include <QObject>

class QComboBox;
class PositionModelSwitch;
class OrientationModelSwitch;
class MixedModelSwitch;
class PositionFilteringController;

class MetaController : public QObject
{
public:
    MetaController(QComboBox * meta_cb, PositionModelSwitch & pos_sw, OrientationModelSwitch & ori_sw,
                   MixedModelSwitch & mix_sw, PositionFilteringController & pos_ctrl);

public slots:
    void configure_control(int cb_idx);

private:
    PositionModelSwitch & pos_sw;
    OrientationModelSwitch & ori_sw;
    MixedModelSwitch & mix_sw;
    PositionFilteringController & pos_ctrl;
};

#endif // METACONTROLLER_H

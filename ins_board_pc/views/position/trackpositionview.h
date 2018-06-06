#ifndef TRACKPOSITIONVIEW_H
#define TRACKPOSITIONVIEW_H

#include "views/IBaseView.h"
#include "views/IAccumView.h"
#include "core/IPositionProvider.h"
#include "qualitycontrol.h"

class QLineEdit;

struct TrackPositionView : IPositionView, IAccumView
{
    TrackPositionView(QLineEdit * track_angle_le, QLineEdit * ground_speed_le);
    ~TrackPositionView() override = default;

    void update(const ViewModel & vm) override;
    void clear();

    void set_accumulator_capacity(std::size_t new_capacity) override;

private:
    QualityControl<Vector3D> speed_accum;

    QLineEdit * track_angle_le;
    QLineEdit * ground_speed_le;
};

#endif // TRACKPOSITIONVIEW_H

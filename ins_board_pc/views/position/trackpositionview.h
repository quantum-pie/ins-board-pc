#ifndef TRACKPOSITIONVIEW_H
#define TRACKPOSITIONVIEW_H

#include "views/base/IBaseView.h"
#include "core/IPositionProvider.h"

class QLineEdit;

struct TrackPositionView : IPositionView
{
    TrackPositionView(QLineEdit * track_angle_le, QLineEdit * ground_speed_le);
    ~TrackPositionView() override = default;

    void update(const ViewModel & vm) override;
    void clear();

private:
    QLineEdit * track_angle_le;
    QLineEdit * ground_speed_le;
};

#endif // TRACKPOSITIONVIEW_H

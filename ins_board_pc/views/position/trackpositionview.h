#ifndef TRACKPOSITIONVIEW_H
#define TRACKPOSITIONVIEW_H

#include "views/IBaseView.h"
#include "core/IPositionProvider.h"
#include "qualitycontrol.h"

#include <QObject>

class QLineEdit;

struct TrackPositionView : QObject, IPositionView
{
    TrackPositionView(QLineEdit * samples_le, QLineEdit * track_angle_le, QLineEdit * ground_speed_le);
    ~TrackPositionView() override = default;

    void update(const ViewModel & vm) override;
    void clear();

public slots:
    void set_accumulator_capacity(const QString & str);

private:
    QualityControl<Vector3D> speed_accum;

    QLineEdit * track_angle_le;
    QLineEdit * ground_speed_le;
};

#endif // TRACKPOSITIONVIEW_H

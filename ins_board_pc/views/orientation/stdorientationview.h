#ifndef RPYTEXTVIEW_H
#define RPYTEXTVIEW_H

#include "views/IBaseView.h"
#include "core/IOrientationProvider.h"
#include "qualitycontrol.h"

class QLineEdit;

struct StdOrientationView : IOrientationView
{
    StdOrientationView(QLineEdit * roll_std_le, QLineEdit * pitch_std_le, QLineEdit * yaw_std_le, QLineEdit * magnetic_heading_le);
    ~StdOrientationView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

    void set_accumulator_capacity(std::size_t new_capacity);

private:
    QualityControl<Vector3D> rpy_ctrl;

    QLineEdit * roll_std_le;
    QLineEdit * pitch_std_le;
    QLineEdit * yaw_std_le;
    QLineEdit * magnetic_heading_le;
};

#endif // RPYTEXTVIEW_H

#ifndef RPYTEXTVIEW_H
#define RPYTEXTVIEW_H

#include "views/IBaseView.h"
#include "core/IOrientationProvider.h"
#include "qualitycontrol.h"

#include <QObject>

class QLineEdit;

class StdOrientationView : public QObject, public IOrientationView
{
    Q_OBJECT

public:
    StdOrientationView(QLineEdit * sample_le, QLineEdit * roll_std_le, QLineEdit * pitch_std_le, QLineEdit * yaw_std_le, QLineEdit * magnetic_heading_le);
    ~StdOrientationView() override = default;

    void update(const ViewModel & pvd) override;
    void clear() override;

public slots:
    void set_accumulator_capacity(const QString & str);

private:
    QualityControl<Vector3D> rpy_ctrl;

    QLineEdit * roll_std_le;
    QLineEdit * pitch_std_le;
    QLineEdit * yaw_std_le;
    QLineEdit * magnetic_heading_le;
};

#endif // RPYTEXTVIEW_H

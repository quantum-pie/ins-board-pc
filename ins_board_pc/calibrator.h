#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include <QVector3D>

class Calibrator
{
public:
    Calibrator();
    void reset();
    void update(const QVector3D & vec);
    QVector3D calibrate(const QVector3D & vec);

    double get_x_bias();
    double get_y_bias();
    double get_z_bias();
    double get_x_scale();
    double get_y_scale();
    double get_z_scale();

private:
    void update_borders(double & lower, double & upper, double val);

    double x_min, x_max, y_min, y_max, z_min, z_max;
    double x_bias, y_bias, z_bias, x_scale, y_scale, z_scale;
};

#endif // CALIBRATOR_H

#include "calibrator.h"

#include <QtGlobal>

Calibrator::Calibrator()
{
    reset();
}

void Calibrator::reset()
{
    x_min = qInf();
    x_max = -qInf();
    y_min = qInf();
    y_max = -qInf();
    z_min = qInf();
    z_max = -qInf();
    x_bias = 0;
    y_bias = 0;
    z_bias = 0;
    x_scale = 1;
    y_scale = 1;
    z_scale = 1;
}

void Calibrator::update(const QVector3D & vec)
{
    update_borders(x_min, x_max, vec.x());
    update_borders(y_min, y_max, vec.y());
    update_borders(z_min, z_max, vec.z());

    x_bias = (x_min + x_max) / 2;
    y_bias = (y_min + y_max) / 2;
    z_bias = (z_min + z_max) / 2;

    double x_diff = x_max - x_min;
    double y_diff = y_max - y_min;
    double z_diff = z_max - z_min;

    double scaler = qMax(qMax(x_diff, y_diff), z_diff);

    x_scale = scaler / x_diff;
    y_scale = scaler / y_diff;
    z_scale = scaler / z_diff;
}

QVector3D Calibrator::calibrate(const QVector3D & vec)
{
    return QVector3D( (vec.x() - x_bias) * x_scale,
                      (vec.y() - y_bias) * y_scale,
                      (vec.z() - z_bias) * z_scale);
}

void Calibrator::update_borders(double & lower, double & upper, double val)
{
    if(val > upper)
        upper = val;
    else if(val < lower)
        lower = val;
}

double Calibrator::get_x_bias()
{
    return x_bias;
}

double Calibrator::get_y_bias()
{
    return y_bias;
}

double Calibrator::get_z_bias()
{
    return z_bias;
}

double Calibrator::get_x_scale()
{
    return x_scale;
}

double Calibrator::get_y_scale()
{
    return y_scale;
}

double Calibrator::get_z_scale()
{
    return z_scale;
}

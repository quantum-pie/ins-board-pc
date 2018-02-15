#ifndef CALIBRATOR_H
#define CALIBRATOR_H

#include "eigenaux.h"

#include <QVector3D>

class Calibrator
{
public:
    Calibrator();
    void reset();
    void update(const NumVector & m);
    void calibrate(NumVector & m) const;
    NumVector calibrate(const NumVector & m) const;
    QVector3D calibrate(const QVector3D & vec) const;
    void save() const;
    void fit();

    NumVector get_bias() const;
    NumMatrix get_scale() const;

private:
    void load();

    NumVector bias;
    NumMatrix scale;

    std::vector<NumVector> meas;
};

#endif // CALIBRATOR_H

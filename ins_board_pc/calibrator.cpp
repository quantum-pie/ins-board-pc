#include "calibrator.h"

#include <QtGlobal>
#include <QFile>
#include <QDataStream>
#include <QtMath>

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <boost/numeric/ublas/assignment.hpp>

#include <QDebug>

Calibrator::Calibrator()
{
    reset();
    load();
}

void Calibrator::reset()
{
    bias = ZeroVector(3);
    scale = IdentityMatrix(3);
}

void Calibrator::update(const NumVector & m)
{
    meas.push_back(m);
}

void Calibrator::fit()
{
    using namespace Eigen;

    MatrixXd D(10, meas.size());

    qDebug() << meas.size();

    for(size_t i = 0; i < meas.size(); ++i)
    {
        double x = meas.at(i)[0];
        double y = meas.at(i)[1];
        double z = meas.at(i)[2];

        D(0, i) = x * x;
        D(1, i) = y * y;
        D(2, i) = z * z;
        D(3, i) = 2 * y * z;
        D(4, i) = 2 * x * z;
        D(5, i) = 2 * x * y;
        D(6, i) = 2 * x;
        D(7, i) = 2 * y;
        D(8, i) = 2 * z;
        D(9, i) = 1;
    }

    auto S = D * D.transpose();

    auto S11 = S.block<6, 6>(0, 0);
    auto S12 = S.block<6, 4>(0, 6);
    auto S21 = S.transpose();
    auto S22 = S.block<4, 4>(6, 6);

    auto S22_inv = S22.inverse();

    Matrix<double, 6, 6> C;
    C <<  -1,  1,  1,  0,  0,  0,
           1, -1,  1,  0,  0,  0,
           1,  1, -1,  0,  0,  0,
           0,  0,  0, -4,  0,  0,
           0,  0,  0,  0, -4,  0,
           0,  0,  0,  0,  0, -4;


    auto E = C.inverse() * (S11 - S12 * S22_inv * S21);

    EigenSolver<Matrix<double, 6, 6>> eigensolver(E);
    auto w = eigensolver.eigenvalues();
    auto v = eigensolver.eigenvectors();

    int idx = -1;
    double max = -qInf();
    for(int i = 0; i < 6; ++i)
    {
        if(w[i].real() > max)
        {
            max = w[i].real();
            idx = i;
        }
    }

    Matrix<double, 6, 1> v_1;
    for(int i = 0; i < 6; ++i)
    {
        v_1[i] = v(i, idx).real();
    }

    if(v_1[0] < 0)
        v_1 = -v_1;

    auto v_2 = - S22_inv * S21 * v_1;

    Matrix<double, 3, 3> M;
    M <<  v_1[0], v_1[3], v_1[4],
          v_1[3], v_1[1], v_1[5],
          v_1[4], v_1[5], v_1[2];

    Vector3d n;
    n << v_2[0], v_2[1], v_2[2];

    double d = v_2[3];

    auto M_inv = M.inverse();

    auto bias_eig = - M_inv * n;
    auto scale_eig = M.sqrt() / qSqrt(n.transpose() * M_inv * n - d);

    for(int i = 0; i < 3; ++i)
    {
        bias[i] = bias_eig[i];
        for(int j = 0; j < 3; ++j)
        {
            scale(i, j) = scale_eig(i, j);
        }
    }
}

NumVector Calibrator::calibrate(const NumVector & m) const
{
    return prod(scale, m - bias);
}

QVector3D Calibrator::calibrate(const QVector3D & vec) const
{
    NumVector m(3);
    m <<= vec.x(), vec.y(), vec.z();
    m = calibrate(m);
    return QVector3D(m[0], m[1], m[2]);
}

void Calibrator::save() const
{
    QString filename = "res/magnet_calib.dat";
    QFile file(filename);
    if(file.open(QIODevice::ReadWrite))
    {
        QDataStream stream(&file);
        stream << bias[0] << bias[1] << bias[2] <<
                    scale(0, 0) << scale(0, 1) << scale(0, 2) <<
                    scale(1, 0) << scale(1, 1) << scale(1, 2) <<
                    scale(2, 0) << scale(2, 1) << scale(2, 2);
        file.close();
    }
}

void Calibrator::load()
{
    QString filename = "res/magnet_calib.dat";
    QFile file(filename);
    if(file.open(QIODevice::ReadWrite))
    {
        QDataStream stream(&file);
        stream >> bias[0] >> bias[1] >> bias[2] >>
                scale(0, 0) >> scale(0, 1) >> scale(0, 2) >>
                scale(1, 0) >> scale(1, 1) >> scale(1, 2) >>
                scale(2, 0) >> scale(2, 1) >> scale(2, 2);
        file.close();
    }
}


NumVector Calibrator::get_bias() const
{
    return bias;
}

NumMatrix Calibrator::get_scale() const
{
    return scale;
}

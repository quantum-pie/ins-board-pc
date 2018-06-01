#include "magncalibrator.h"

#include <fstream>

#include <Eigen/Dense>

#include <boost/lexical_cast.hpp>

const MagnCalibrator::C_type MagnCalibrator::C = MagnCalibrator::initialize_cmatrix();

MagnCalibrator::C_type MagnCalibrator::initialize_cmatrix()
{
    C_type C;
    C <<   -1,  1,  1,  0,  0,  0,
            1, -1,  1,  0,  0,  0,
            1,  1, -1,  0,  0,  0,
            0,  0,  0, -4,  0,  0,
            0,  0,  0,  0, -4,  0,
            0,  0,  0,  0,  0, -4;
    return C;
}

MagnCalibrator::MagnCalibrator()
{
	meas.reserve(buf_size);
    reset();
    load();
}

void MagnCalibrator::reset()
{
	bias = Vector3D::Zero();
	scale = Matrix3D::Identity();
}

void MagnCalibrator::update(const Vector3D & m)
{
    meas.push_back(m);
}

void MagnCalibrator::fit()
{
    DynamicMatrix D(10, meas.size());

    double x, y, z;
    for(size_t i = 0; i < meas.size(); ++i)
    {
        x = meas.at(i)[0];
        y = meas.at(i)[1];
        z = meas.at(i)[2];

        D(0, i) = x * x;
        D(1, i) = y * y;
        D(2, i) = z * z;
        D(3, i) = 2 * y * z;
        D(4, i) = 2 * x * z;
        D(5, i) = 2 * x * y;
        D(6, i) = x;
        D(7, i) = y;
        D(8, i) = z;
        D(9, i) = 1;
    }

    auto S = D * D.transpose();

    auto S11 = S.block<6, 6>(0, 0);
    auto S12 = S.block<6, 4>(0, 6);
    auto S21 = S12.transpose();
    auto S22 = S.block<4, 4>(6, 6);

    auto S22_inv = S22.inverse();

    auto E = C.inverse() * (S11 - S12 * S22_inv * S21);

    Eigen::EigenSolver<C_type> eigensolver(E);
    auto w = eigensolver.eigenvalues();
    auto v = eigensolver.eigenvectors();

    int idx = -1;
    double max = -std::numeric_limits<double>::infinity();
    for(int i = 0; i < 6; ++i)
    {
        if(w[i].real() > max)
        {
            max = w[i].real();
            idx = i;
        }
    }

    StaticVector<6> v_1;
    for(int i = 0; i < 6; ++i)
    {
        v_1[i] = v(i, idx).real();
    }

    if(v_1[0] < 0)
    {
        v_1 = -v_1;
    }

    auto v_2 = - S22_inv * S21 * v_1;

    Matrix3D M;
    M <<  v_1[0], v_1[5], v_1[4],
          v_1[5], v_1[1], v_1[3],
          v_1[4], v_1[3], v_1[2];

    Vector3D n;
    n << v_2[0], v_2[1], v_2[2];

    double d = v_2[3];

    Matrix3D M_inv = M.inverse();

    bias = -M_inv * n / 2;

    Matrix3D tmp = M.llt().matrixU();

    scale = tmp / std::sqrt(0.25 * n.transpose() * M_inv * n - d);
}

MagnCalibrator::meas_iter MagnCalibrator::meas_begin() const
{
    return meas.cbegin();
}

MagnCalibrator::meas_iter MagnCalibrator::meas_end() const
{
    return meas.cend();
}

void MagnCalibrator::clear_meas()
{
    meas.resize(0);
}

void MagnCalibrator::fit_simple()
{
    Vector3D minimums = Vector3D::Constant(std::numeric_limits<double>::infinity());
    Vector3D maximums = -minimums;

    for(auto & m : meas)
    {
        for(std::size_t i = 0; i < 3; ++i)
        {
            if(m[i] < minimums[i])
            {
                minimums[i] = m[i];
            }
            else if(m[i] > maximums[i])
            {
                maximums[i] = m[i];
            }
        }
    }

    bias = (minimums + maximums) / 2;

    Vector3D span = maximums - minimums;
    scale.diagonal() = 2 * span.cwiseInverse();

    meas.resize(0);
}

Vector3D MagnCalibrator::calibrate(const Vector3D & m) const
{
    return scale * (m - bias);
}

void MagnCalibrator::load()
{
    std::ifstream ifile("res/magnet_calib.dat", std::ios::binary);

    if(ifile.is_open())
    {
    	ifile.read(reinterpret_cast<char *>(&bias[0]), value_size);
    	ifile.read(reinterpret_cast<char *>(&bias[1]), value_size);
    	ifile.read(reinterpret_cast<char *>(&bias[2]), value_size);

    	ifile.read(reinterpret_cast<char *>(&scale(0, 0)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(0, 1)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(0, 2)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(1, 0)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(1, 1)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(1, 2)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(2, 0)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(2, 1)), value_size);
    	ifile.read(reinterpret_cast<char *>(&scale(2, 2)), value_size);

        ifile.close();
    }
}

void MagnCalibrator::save() const
{
    std::ofstream ofile("res/magnet_calib.dat", std::ios::binary);

    if(ofile.is_open())
    {
    	ofile.write(reinterpret_cast<const char *>(&bias[0]), value_size);
    	ofile.write(reinterpret_cast<const char *>(&bias[1]), value_size);
    	ofile.write(reinterpret_cast<const char *>(&bias[2]), value_size);

    	ofile.write(reinterpret_cast<const char *>(&scale(0, 0)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(0, 1)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(0, 2)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(1, 0)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(1, 1)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(1, 2)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(2, 0)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(2, 1)), value_size);
    	ofile.write(reinterpret_cast<const char *>(&scale(2, 2)), value_size);

    	ofile.close();
    }
}

Vector3D MagnCalibrator::get_bias() const
{
    return bias;
}

Matrix3D MagnCalibrator::get_scale() const
{
    return scale;
}

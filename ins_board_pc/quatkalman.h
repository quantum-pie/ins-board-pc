#ifndef QUATKALMAN_H
#define QUATKALMAN_H

#include <QDate>

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

#include "wmmwrapper.h"

namespace ublas = boost::numeric::ublas;
using NumMatrix = ublas::matrix<double>;
using NumVector = ublas::vector<double>;
using ZeroMatrix = ublas::zero_matrix<double>;
using IdentityMatrix = ublas::identity_matrix<double>;

class QuaternionKalman
{
public:
    struct KalmanInput
    {
        NumVector w;
        NumVector a;
        NumVector m;
        QDate day;
        NumVector pos;
        //double lat;
        //double lon;
        //double alt;
        double vel;
        double dt;
    };

    QuaternionKalman(double proc_gyro_std, double proc_gyro_bias_std,
                     double proc_accel_std, double meas_accel_std,
                     double meas_magn_std, double meas_cep,
                     double meas_vel_std);

    void step(const KalmanInput & z);

    void reset();

    NumVector get_state();
    NumVector get_orinetation_quaternion();
    NumVector get_gyro_bias();
    NumVector get_position();
    NumVector get_velocity();
    NumVector get_acceleration();

private:    
    void update(const KalmanInput & z);
    void initialize(const KalmanInput & z1);

    /* create Kalman matrices */
    NumMatrix create_quat_bias_mtx(double dt_2);
    NumMatrix create_transition_mtx(const KalmanInput & z);
    NumMatrix create_proc_noise_cov_mtx(double dt);
    NumMatrix create_meas_noise_cov_mtx(double lat, double lon);
    NumMatrix create_meas_proj_mtx(const KalmanInput & z, const NumVector & predicted_z);

    /* auxiliary transformations */
    NumMatrix quaternion_to_dcm(const NumVector & quaternion);
    NumMatrix geodetic_to_dcm(double lat, double lon);
    NumVector expected_mag(double lat, double lon, double alt, QDate day);

    /* auxiliary derivatives */
    NumMatrix ddcm_dqs(const NumVector & quaternion);
    NumMatrix ddcm_dqx(const NumVector & quaternion);
    NumMatrix ddcm_dqy(const NumVector & quaternion);
    NumMatrix ddcm_dqz(const NumVector & quaternion);

    NumMatrix dcm_lat_partial(double lat, double lon);
    NumMatrix dcm_lon_partial(double lat, double lon);

    /* main derivatives */
    NumMatrix dgeo_dpos(double lat, double lon, double alt);

    /* from state to measurements */
    void calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                                 double lat, double lon,
                                 double & ax, double & ay, double & az);

    void calculate_magnetometer(const NumVector & orientation_quat, double magnitude,
                                double lat, double lon, double alt, QDate day,
                                double & mx, double & my, double & mz);

    void calculate_geodetic(const NumVector & position,
                            double & lat, double & lon, double & alt);

    void calculate_velocity(const NumVector & velocity, double & vel);

    WrapperWMM wmm;

    KalmanInput z0;

    NumVector x;
    NumMatrix P;

    bool initialized;
    bool first_sample_received;

    double proc_gyro_std;
    double proc_gyro_bias_std;
    double proc_accel_std;
    double meas_accel_std;
    double meas_magn_std;
    double meas_cep;
    double meas_vel_std;
};

#endif // QUATKALMAN_H

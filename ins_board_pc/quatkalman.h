#ifndef QUATKALMAN_H
#define QUATKALMAN_H

#include <boost/numeric/ublas/vector.hpp>
#include <boost/numeric/ublas/matrix.hpp>

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
        double lat;
        double lon;
        double alt;
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

    NumMatrix create_quat_bias_mtx(double dt_2);
    NumMatrix create_transition_mtx(const KalmanInput & z);
    NumMatrix create_proc_noise_cov_mtx(const KalmanInput & z);
    NumMatrix create_meas_noise_cov_mtx(const KalmanInput & z);
    NumMatrix create_meas_proj_mtx(const KalmanInput & z, const NumVector & predicted_z);

    NumMatrix quaternion_to_dcm(NumVector & quaternion);
    NumMatrix geodetic_to_dcm(double lat, double lon);

    NumMatrix ddcm_dqs(NumVector & quaternion);
    NumMatrix ddcm_dqx(NumVector & quaternion);
    NumMatrix ddcm_dqy(NumVector & quaternion);
    NumMatrix ddcm_dqz(NumVector & quaternion);

    NumMatrix dcm_lat_partial(double lat, double lon);
    NumMatrix dcm_lon_partial(double lat, double lon);

    NumMatrix dgeo_dpos(double lat, double lon, double alt);

    void calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                                 double lat, double lon,
                                 double & ax, double & ay, double & az);

    void calculate_magnetometer(const NumVector & orientation_quat,
                                double & mx, double & my, double & mz);

    void calculate_geodetic(const NumVector & position,
                            double & lat, double & lon, double & alt);

    void calculate_velocity(const NumVector & velocity, double & vel);

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

    const double earth_rad = 6371e3;
    const double a = 6378137;
    const double b = 6356752.3142;
    const double e_2 = (a * a - b * b) / (a * a);
};

#endif // QUATKALMAN_H

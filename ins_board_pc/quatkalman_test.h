#ifndef QUATKALMAN_TEST_H
#define QUATKALMAN_TEST_H

#include "abstractorientationfilter.h"
#include "abstractpositionfilter.h"

class QuatKalmanTest final : public AbstractOrientationFilter, public AbstractPositionFilter
{
public:
    struct ProcessNoiseParams
    {
        double gyro_std;
        double gyro_bias_std;
        double accel_std;
    };

    struct MeasurementNoiseParams
    {
        double accel_std;
        double magn_std;
        double gps_cep;
        double gps_vel_abs_std;
    };

    struct InitCovParams
    {
        double quat_std;
        double bias_std;
        double pos_std;
        double vel_std;
        double accel_std;
    };

    struct FilterParams
    {
        ProcessNoiseParams proc_params;
        MeasurementNoiseParams meas_params;
        InitCovParams init_params;
        int accum_capacity;
    };

    QuatKalmanTest(const FilterParams & params);
    ~QuatKalmanTest() override;

    void step(const FilterInput & z) override;
    void reset() override;

    NumVector get_orientation_quaternion() override;
    NumVector get_gyro_bias() override;
    NumVector get_position() override;
    NumVector get_velocity() override;
    NumVector get_acceleration() override;

    void get_rpy(double & roll, double & pitch, double & yaw) override;
    void get_geodetic(double & lat, double & lon, double & alt) override;

    void set_proc_gyro_std(double std);
    void set_proc_gyro_bias_std(double std);
    void set_proc_accel_std(double std);

    void set_meas_accel_std(double std);
    void set_meas_magn_std(double std);
    void set_meas_pos_std(double std);
    void set_meas_vel_std(double std);

    void set_init_quat_std(double std);
    void set_init_bias_std(double std);
    void set_init_pos_std(double std);
    void set_init_vel_std(double std);
    void set_init_accel_std(double std);

protected:
    void update(const FilterInput & z) override;
    void accumulate(const FilterInput & z) override;
    void initialize(const FilterInput & z) override;
    void normalize_state() override;

private:
    /* create Kalman matrices */
    NumMatrix create_transition_mtx(const FilterInput & z);
    NumMatrix create_proc_noise_cov_mtx(double dt);
    NumMatrix create_meas_noise_cov_mtx(double lat, double lon, double magn_mag);
    NumMatrix create_meas_proj_mtx(double lat, double lon, double alt, QDate day, const NumVector & v);

    /* from state to measurements */
    void calculate_accelerometer(const NumVector & orientation_quat, const NumVector & acceleration,
                                 double lat, double lon, double alt,
                                 double & ax, double & ay, double & az);

    void calculate_magnetometer(const NumVector & orientation_quat,
                                double lat, double lon, double alt, QDate day,
                                double & mx, double & my, double & mz);

    void calculate_geodetic(const NumVector & position,
                            double & lat, double & lon, double & alt);

    void calculate_velocity(const NumVector & velocity, double & vel);

    static const int state_size = 16;
    static const int measurement_size = 10;

    NumMatrix P;

    FilterParams params;
};

#endif // QUATKALMAN_TEST_H
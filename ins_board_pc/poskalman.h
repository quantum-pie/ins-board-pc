#ifndef POSKALMAN_H
#define POSKALMAN_H

#include "abstractkalmanpositionfilter.h"

class PositionKalman final : public AbstractKalmanPositionFilter
{
public:
    struct ProcessNoiseParams
    {
        double accel_std;
    };

    struct MeasurementNoiseParams
    {
        double gps_cep;
        double gps_vel_abs_std;
    };

    struct InitCovParams
    {
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

    PositionKalman(const FilterParams & params);
    ~PositionKalman() override;

    void step(const FilterInput & z) override;

    NumVector get_position() const override;
    NumVector get_velocity() const override;
    NumVector get_acceleration() const override;

    void get_geodetic(double & lat, double & lon, double & alt) const override;

    void set_proc_accel_std(double std) override;

    void set_meas_pos_std(double std) override;
    void set_meas_vel_std(double std) override;

    void set_init_pos_std(double std) override;
    void set_init_vel_std(double std) override;
    void set_init_accel_std(double std) override;

protected:
    void update(const FilterInput & z) override;
    void initialize(const FilterInput & z) override;
    void accumulate(const FilterInput & z) override;

private:
    /* create Kalman matrices */
    NumMatrix create_transition_mtx(const FilterInput & z) const;
    NumMatrix create_proc_noise_cov_mtx(double dt) const;
    NumMatrix create_meas_noise_cov_mtx(double lat, double lon) const;
    NumMatrix create_meas_proj_mtx(const NumVector & v) const;

    void calculate_geodetic(const NumVector & position,
                            double & lat, double & lon, double & alt) const;

    void calculate_velocity(const NumVector & velocity, double & vel) const;

    static const int state_size;
    static const int measurement_size;

    NumMatrix P;
    FilterParams params;
};

#endif // POSKALMAN_H

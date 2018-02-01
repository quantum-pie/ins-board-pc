#ifndef QUALITYCONTROL_H
#define QUALITYCONTROL_H

#include <boost/circular_buffer.hpp>

class QualityControl
{
public:
    QualityControl(size_t buf_size = 200);
    void update(double val);
    double get_mean();
    double get_std();
    bool is_saturated();
    void set_sampling(size_t samples);
    size_t get_sampling();

private:
    size_t buf_size;
    boost::circular_buffer<double> buf;
    double mean;
    double std;
};

#endif // QUALITYCONTROL_H

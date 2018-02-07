#ifndef QUALITYCONTROL_H
#define QUALITYCONTROL_H

#include <boost/circular_buffer.hpp>

class QualityControl
{
public:
    QualityControl(std::size_t buf_size = 200);
    void update(double val);
    double get_mean();
    double get_std();
    bool is_saturated();
    void set_sampling(std::size_t samples);
    std::size_t get_sampling();
    void reset();

private:
    std::size_t buf_size;
    boost::circular_buffer<double> buf;
    double mean;
    double std;
};

#endif // QUALITYCONTROL_H

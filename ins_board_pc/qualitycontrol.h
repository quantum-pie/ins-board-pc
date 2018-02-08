#ifndef QUALITYCONTROL_H
#define QUALITYCONTROL_H

#include <boost/circular_buffer.hpp>

class QualityControl
{
public:
    QualityControl(std::size_t buf_size = 200);
    void update(double val);
    double get_mean() const;
    double get_std() const;
    bool is_saturated() const;
    void set_sampling(std::size_t samples);
    std::size_t get_sampling() const;
    void reset();

private:
    std::size_t buf_size;
    boost::circular_buffer<double> buf;
    double mean;
    double std;
};

#endif // QUALITYCONTROL_H

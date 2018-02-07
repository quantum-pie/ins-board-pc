#include "qualitycontrol.h"

#include <QtMath>

QualityControl::QualityControl(std::size_t buf_size) : buf_size(buf_size), buf(buf_size)
{

}

void QualityControl::update(double val)
{
    buf.push_back(val);

    double s1 = 0;
    double s2 = 0;

    for(std::size_t i = 0; i < buf.size(); ++i)
    {
        s1 += buf[i];
        s2 += buf[i] * buf[i];
    }

    mean = s1 / buf.size();
    std = qSqrt(s2 / buf.size() - mean * mean);
}

double QualityControl::get_mean()
{
    return mean;
}

double QualityControl::get_std()
{
    return std;
}

bool QualityControl::is_saturated()
{
    return buf.size() == buf_size;
}

void QualityControl::set_sampling(std::size_t samples)
{
    buf.set_capacity(samples);
    buf_size = samples;
}

std::size_t QualityControl::get_sampling()
{
    return buf_size;
}

void QualityControl::reset()
{
    buf.clear();
}


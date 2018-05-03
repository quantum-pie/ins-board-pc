/*! \file qualitycontrol.h
  */

#ifndef QUALITYCONTROL_H
#define QUALITYCONTROL_H

#include <boost/circular_buffer.hpp>
#include <cmath>

/*!
 * @brief Estimate quality control class.
 *
 * This class allows to measure basic estimates metrics: bias and standard deviation.
 * @tparam T estimate value type.
 */
template<typename T>
class QualityControl
{
public:
    /*!
     * @brief Class constructor.
     * @param buf_size size of buffer.
     * @param zero estimate value zero.
     */
    explicit QualityControl(std::size_t buf_size = 200, const T & zero = T{})
    	: buf_size{buf_size}, buf{buf_size}, zero{zero} {}

    /*!
     * @brief Take new estimate.
     * @param val new estimate.
     */
    void update(const T & val)
    {
        buf.push_back(val);
    }

    /*!
     * @brief Get estimate bias (mean).
     * @return estimate bias (mean).
     */
    T get_mean() const
    {
    	T s { zero };
        for(std::size_t i = 0; i < buf.size(); ++i)
        {
            s += buf[i];
        }
        return s / buf.size();
    }

    /*!
     * @brief Get estimate standard deviation.
     * @return estimate standard deviation.
     */
    T get_std() const
    {
        T s1 { zero }, s2 { zero };
        for(std::size_t i = 0; i < buf.size(); ++i)
        {
            s1 += buf[i];
            s2 += buf[i] * buf[i];
        }

        T mean = s1 / buf.size();
        return std::sqrt(s2 / buf.size() - mean * mean);
    }

    /*!
     * @brief Check if quality parameters are reliable.
     * @return true of parameters are reliable.
     */
    bool is_saturated() const
    {
    	return buf.size() == buf_size;
    }

    /*!
     * @brief Set internal buffer size.
     * @param samples size of internal buffer.
     */
    void set_sampling(std::size_t samples)
    {
        buf.set_capacity(samples);
        buf_size = samples;
    }

    /*!
     * @brief Get internal buffer size.
     * @return size of internal buffer.
     */
    std::size_t get_sampling() const
    {
    	return buf_size;
    }

    /*!
     * @brief Reset internal buffer.
     */
    void reset()
    {
    	buf.clear();
    }

private:
    std::size_t buf_size;                //!< Size of internal buffer.
    boost::circular_buffer<T> buf;       //!< Internal circular buffer.
    const T zero;                        //!< Zero value.
};

#endif // QUALITYCONTROL_H

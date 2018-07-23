/*! \file qualitycontrol.h
  */

#ifndef QUALITYCONTROL_H
#define QUALITYCONTROL_H

#include "eigenaux.h"

#include <boost/circular_buffer.hpp>
#include <cmath>
#include <numeric>
#include <type_traits>

/*!
 * @brief Estimate quality control class.
 *
 * This class allows to measure basic estimates metrics of scalar and vector arithmetic types: bias and standard deviation.
 * @tparam T estimate value type.
 */
template<typename T>
class QualityControl
{
public:
    /*!
     * @brief Class constructor.
     * @param buf_size size of buffer.
     */
    explicit QualityControl(std::size_t buf_size = 200)
    	: buf_size{buf_size}, buf{buf_size} {}

    /*!
     * @brief Take new estimate.
     * @param val new estimate.
     */
    void update(const T & val)
    {
        buf.push_back(val);
    }

    /*!
     * @brief Get estimate bias of vector type (mean).
     * @tparam U estimated type.
     * @return estimate bias (mean).
     */
    template<typename U = T>
    std::enable_if_t<std::is_base_of<Eigen::MatrixBase<U>, T>::value &&
                     std::is_arithmetic<typename U::value_type>::value, T>
    get_mean() const
    {
    	T s { T::Zero() };
    	return std::accumulate(buf.begin(), buf.end(), s) / buf.size();
    }

    /*!
     * @brief Get estimate bias of scalar type (mean).
     * @tparam U estimated type.
     * @return estimate bias (mean).
     */
    template<typename U = T>
    std::enable_if_t<std::is_arithmetic<U>::value, T>
    get_mean() const
    {
    	return std::accumulate(buf.begin(), buf.end(), 0) / buf.size();
    }

    /*!
     * @brief Get estimate standard deviation of vector type.
     * @tparam U estimated type.
     * @return estimate standard deviation.
     */
    template<typename U = T>
    std::enable_if_t<std::is_base_of<Eigen::MatrixBase<U>, T>::value &&
                     std::is_arithmetic<typename U::value_type>::value, T>
    get_std() const
    {
        T s1 { T::Zero() }, s2 { T::Zero() };
        for(std::size_t i = 0; i < buf.size(); ++i)
        {
            s1 += buf[i];
            s2 += buf[i].cwiseProduct(buf[i]);
        }

        T mean = s1 / buf.size();
        return (s2 / buf.size() - mean.cwiseProduct(mean)).cwiseSqrt();
    }

    /*!
     * @brief Get estimate standard deviation of scalar type.
     * @tparam U estimated type.
     * @return estimate standard deviation.
     */
    template<typename U = T>
    std::enable_if_t<std::is_arithmetic<U>::value, T>
    get_std() const
    {
        T s1 { 0 }, s2 { 0 };
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
};

#endif // QUALITYCONTROL_H

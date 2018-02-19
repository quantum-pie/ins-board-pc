/*! \file qualitycontrol.h
  */

#ifndef QUALITYCONTROL_H
#define QUALITYCONTROL_H

#include <boost/circular_buffer.hpp>

/*!
 * \brief Estimate quality control class.
 *
 * This class allows to measure basic estimates metrics: bias and standard deviation.
 */
class QualityControl
{
public:
    /*!
     * \brief Constructor.
     * \param buf_size size of internal accumulation buffer.
     */
    QualityControl(std::size_t buf_size = 200);

    /*!
     * \brief Take new estimate and update quality parameters.
     * \param val new estimate.
     */
    void update(double val);

    /*!
     * \brief Get estimate bias.
     * \return estimate bias.
     */
    double get_mean() const;

    /*!
     * \brief Get estimate standard deviation.
     * \return estimate standard deviation.
     */
    double get_std() const;

    /*!
     * \brief Check if quality parameters are reliable.
     * \return true of parameters are reliable.
     */
    bool is_saturated() const;

    /*!
     * \brief Set internal buffer size.
     * \param samples size of internal buffer.
     */
    void set_sampling(std::size_t samples);

    /*!
     * \brief Get internal buffer size.
     * \return size of internal buffer.
     */
    std::size_t get_sampling() const;

    /*!
     * \brief Reset internal buffer.
     */
    void reset();

private:
    std::size_t buf_size;                   //!< Size of internal buffer.
    boost::circular_buffer<double> buf;     //!< Internal circular buffer.
    double mean;                            //!< Estimate bias.
    double std;                             //!< Estimate standard deviation.
};

#endif // QUALITYCONTROL_H

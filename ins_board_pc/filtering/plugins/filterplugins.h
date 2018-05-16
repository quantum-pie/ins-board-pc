#ifndef FILTERPLUGINS_H
#define FILTERPLUGINS_H

class FilterInput;

/*!
 * @brief Kalman filter extrapolator CRTP mixin base class.
 * @tparam Derived derived extrapolator.
 */
template<typename Derived>
struct IExtrapolator
{
    //! Derived extrapolator alias.
    using exptrapolator_base = Derived;

    /*!
     * @brief Run extrapolation.
     * @param z filter input.
     */
    void extrapolate(const FilterInput & z)
    {
        static_cast<Derived&>(*this).do_extrapolate(z);
    }
};

/*!
 * @brief Kalman filter corrector CRTP mixin base class.
 * @tparam Derived derived corrector.
 */
template<typename Derived>
struct ICorrector
{
    //! Derived corrector alias.
    using corrector_base = Derived;

    /*!
     * @brief Run correction.
     * @param z filter input.
     */
    void correct(const FilterInput & z)
    {
        static_cast<Derived&>(*this).do_correct(z);
    }
};

#endif // FILTERPLUGINS_H

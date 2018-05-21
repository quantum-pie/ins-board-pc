#ifndef ICOMPLEMENTORIENTATIONATTRSETCORE_H
#define ICOMPLEMENTORIENTATIONATTRSETCORE_H

/*!
 * @brief Complementary orientation filter interface.
 */
struct IComplementOrientationAttrSetCore
{
    /*!
     * @brief Class destructor.
     */
    virtual ~IComplementOrientationAttrSetCore() = default;

    virtual void do_set_static_accel_gain(double gain) = 0;
    virtual void do_set_static_magn_gain(double gain) = 0;
    virtual void do_set_bias_gain(double gain) = 0;
};

#endif // ICOMPLEMENTORIENTATIONATTRSETCORE_H

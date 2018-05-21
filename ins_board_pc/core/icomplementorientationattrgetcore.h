#ifndef ICOMPLEMENTORIENTATIONATTRGETCORE_H
#define ICOMPLEMENTORIENTATIONATTRGETCORE_H

struct IComplementOrientationAttrGetCore
{
    /*!
     * @brief Class destructor.
     */
    virtual ~IComplementOrientationAttrGetCore() = default;

    virtual double do_get_static_accel_gain() const = 0;
    virtual double do_get_static_magn_gain() const = 0;
    virtual double do_get_bias_gain() const = 0;
};

#endif // ICOMPLEMENTORIENTATIONATTRGETCORE_H

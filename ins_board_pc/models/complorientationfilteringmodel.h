#ifndef COMPLORIENTATIONFILTERINGMODEL_H
#define COMPLORIENTATIONFILTERINGMODEL_H

#include "models/orientationfilteringmodel.h"
#include "filtering/public_interfaces/IComplementOrientationFilter.h"
#include "core/IComplementOrientationAttrGetCore.h"
#include "core/IComplementOrientationAttrSetCore.h"

class ComplOrientationFilteringModel : public OrientationFilteringModel,
                                        IComplementOrientationAttrGetCore,
                                        IComplementOrientationAttrSetCore
{
public:
    explicit ComplOrientationFilteringModel(IComplementOrientationFilter * wrapee)
        : OrientationFilteringModel{ wrapee }, wrapee{ wrapee } {}

    ~ComplOrientationFilteringModel() override = default;

    void set_static_accel_gain(double gain)
    {
        do_set_static_accel_gain(gain);
    }

    void set_static_magn_gain(double gain)
    {
        do_set_static_magn_gain(gain);
    }

    void set_bias_gain(double gain)
    {
        do_set_bias_gain(gain);
    }

    double get_static_accel_gain() const
    {
        return do_get_static_accel_gain();
    }

    double get_static_magn_gain() const
    {
        return do_get_static_magn_gain();
    }

    double get_bias_gain() const
    {
        return do_get_bias_gain();
    }

private:
    void do_set_static_accel_gain(double gain) override
    {
        wrapee->set_static_accel_gain(gain);
    }

    void do_set_static_magn_gain(double gain) override
    {
        wrapee->set_static_magn_gain(gain);
    }

    void do_set_bias_gain(double gain) override
    {
        wrapee->set_bias_gain(gain);
    }

    double do_get_static_accel_gain() const override
    {
        return wrapee->get_static_accel_gain();
    }

    double do_get_static_magn_gain() const override
    {
        return wrapee->get_static_magn_gain();
    }

    double do_get_bias_gain() const override
    {
        return wrapee->get_bias_gain();
    }

    IComplementOrientationFilter * wrapee;
};

#endif // COMPLORIENTATIONFILTERINGMODEL_H

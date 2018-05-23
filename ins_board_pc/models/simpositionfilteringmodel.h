#ifndef SIMPOSITIONFILTERINGMODEL_H
#define SIMPOSITIONFILTERINGMODEL_H

#include "models/positionfilteringmodel.h"
#include "filtering/public_interfaces/ISimPositionFilter.h"
#include "core/ISimPositionAttrGetCore.h"
#include "core/ISimPositionAttrSetCore.h"

class SimPositionFilteringModel : public PositionFilteringModel,
                                    ISimPositionAttrGetCore,
                                    ISimPositionAttrSetCore
{
public:
    explicit SimPositionFilteringModel(ISimPositionFilter * wrapee)
        : SimPositionFilteringModel{ wrapee }, wrapee{ wrapee } {}

    ~SimPositionFilteringModel() override = default;

    void set_initial_track(double radians)
    {
        do_set_initial_track(radians);
    }

    void set_speed(double ms)
    {
        do_set_speed(ms);
    }

    double get_initial_track() const
    {
        return do_get_initial_track();
    }

    double get_speed() const
    {
        return do_get_speed();
    }

private:
    void do_set_initial_track(double radians) override
    {
        wrapee->set_initial_track(radians);
    }

    void do_set_speed(double ms) override
    {
        wrapee->set_speed(ms);
    }

    double do_get_initial_track() const override
    {
        return wrapee->get_initial_track();
    }

    double do_get_speed() const override
    {
        return wrapee->get_speed();
    }

    ISimPositionFilter * wrapee;
};

#endif // SIMPOSITIONFILTERINGMODEL_H

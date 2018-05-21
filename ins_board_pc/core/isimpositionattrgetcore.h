#ifndef ISIMPOSITIONATTRGETCORE_H
#define ISIMPOSITIONATTRGETCORE_H

struct ISimPositionAttrGetCore
{
    virtual ~ISimPositionAttrGetCore() = default;

    virtual double do_get_initial_track() const = 0;
    virtual double do_get_speed() const = 0;
};

#endif // ISIMPOSITIONATTRGETCORE_H

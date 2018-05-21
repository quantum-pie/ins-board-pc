#ifndef ISIMPOSITIONATTRSETCORE_H
#define ISIMPOSITIONATTRSETCORE_H

struct ISimPositionAttrSetCore
{
    virtual ~ISimPositionAttrSetCore() = default;

    virtual void do_set_initial_track(double radians) = 0;
    virtual void do_set_speed(double ms) = 0;
};

#endif // ISIMPOSITIONATTRSETCORE_H

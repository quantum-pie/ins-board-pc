#ifndef IACCUMVIEW_H
#define IACCUMVIEW_H

#include <cstddef>

#include <QDebug>

struct IAccumView
{
    virtual ~IAccumView() = default;
    virtual void set_accumulator_capacity(std::size_t new_capacity)
    {
        qDebug() << "Set capacity\n";
    }
};

#endif // IACCUMVIEW_H

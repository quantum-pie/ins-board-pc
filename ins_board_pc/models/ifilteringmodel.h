#ifndef IFILTERINGMODEL_H
#define IFILTERINGMODEL_H

#include "core/IFilterCore.h"
#include "filtering/public_interfaces/IFilter.h"

#include <QObject>

class IFilter;

struct IFilteringModel : QObject, private IFilterCore
{       
    /*!
     * @brief Make step.
     * @param z filter input.
     */
    void step(const FilterInput & z)
    {
        do_step(z);
        emit model_updated();
    }

    /*!
     * @brief Reset filter to default state.
     */
    void reset()
    {
        do_reset();
    }

    /*!
     * @brief Class desctructor.
     */
    virtual ~IFilteringModel() = default;

signals:
    void model_updated();

private:
    Q_OBJECT
};

#endif // IFILTERINGMODEL_H

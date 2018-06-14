#ifndef ATTRCONTROLLERBASE_H
#define ATTRCONTROLLERBASE_H

#include "controllers/controllerbase.h"

#include <QString>

template<typename Model>
struct AttrControllerBase : ControllerBase<Model>
{
    using model_setter = void (Model::*)(double);
    void call_setter(const QString & str, model_setter f)
    {
        if(this->model_is_set())
        {
            bool conv_ok;
            double val = str.toDouble(&conv_ok);
            if(conv_ok)
            {
                (this->get_model()->*f)(val);
            }
        }
    }
};

#endif // ATTRCONTROLLERBASE_H

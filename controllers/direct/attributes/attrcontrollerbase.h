#ifndef ATTRCONTROLLERBASE_H
#define ATTRCONTROLLERBASE_H

#include "controllers/controllerbase.h"

#include <QString>

/*!
 * @brief Attributes controller base.
 * This is base implementation of filtering attributes controllers.
 *
 * @tparam Model type of controlled model.
 */
template<typename Model>
struct AttrControllerBase : ControllerBase<Model>
{
    //! Pointer to model attribute setter method alias.
    using model_setter = void (Model::*)(double);

    /*!
     * @brief Call model attribute setter.
     * @param str attribute value string.
     * @param f attribute setter.
     */
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

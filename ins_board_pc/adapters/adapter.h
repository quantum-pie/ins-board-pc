#ifndef ADAPTER_H
#define ADAPTER_H

#include <type_traits>

/*!
 * @brief Adapter class for filtering controllers.
 *
 * This class is designed to adapt input data from different sources
 * for the uniform usage in filtering controllers.
 * @tparam Input input data type.
 * @tparam Output output data type.
 */
template<typename Input, typename Output>
struct Adapter
{
    /*!
     * @brief Specialized dummy converter if Input and Output are from the same hierarchy.
     * @param input input data.
     * @return output data.
     */
    std::enable_if_t<std::is_base_of<Output, Input>::value, Output>
    operator()(const Input & input)
    {
        return input;
    }
};

#endif

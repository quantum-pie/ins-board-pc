#ifndef ADAPTER_H
#define ADAPTER_H

#include <type_traits>

template<typename Input, typename Output>
struct Adapter
{
    std::enable_if_t<std::is_base_of<Output, Input>::value ||
                     std::is_same<Output, Input>::value, Output>
    operator()(const Input & input)
    {
        return input;
    }
};

#endif

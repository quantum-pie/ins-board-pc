#ifndef FILTERPLUGINS_H
#define FILTERPLUGINS_H

class FilterInput;

template<typename Derived>
struct IExtrapolator
{
    void extrapolate(const FilterInput & z)
    {
        static_cast<Derived&>(*this).do_extrapolate();
    }
};

template<typename Derived>
struct ICorrector
{
    void correct(const FilterInput & z)
    {
        static_cast<Derived&>(*this).do_correct();
    }
};

#endif // FILTERPLUGINS_H

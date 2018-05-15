#ifndef FILTERPLUGINS_H
#define FILTERPLUGINS_H

class FilterInput;

template<typename Derived>
struct IExtrapolator
{
    using exptrapolator_base = Derived;

    void extrapolate(const FilterInput & z)
    {
        static_cast<Derived&>(*this).do_extrapolate(z);
    }
};

template<typename Derived>
struct ICorrector
{
    using corrector_base = Derived;

    void correct(const FilterInput & z)
    {
        static_cast<Derived&>(*this).do_correct(z);
    }
};

#endif // FILTERPLUGINS_H

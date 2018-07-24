#ifndef UTPARAMS_H
#define UTPARAMS_H

/*!
 * @brief Parameters of unscented transform.
 */
struct UnscentedTransformParams
{
    double kappa;       //!< Kappa.
    double beta;        //!< Beta.
    double alpha;       //!< Alpha.

    static const UnscentedTransformParams default_params;   //!< Default UT params.
};

#endif // UTPARAMS_H

/*
 * quatfwd.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_QUATFWD_H_
#define INCLUDE_QUATFWD_H_

#include "eigenaux.h"

namespace quat
{

class Quaternion;

using skew_type = StaticMatrix<4, 4>;
using delta_type = StaticMatrix<4, 3>;
using vector_form = StaticVector<4>;

}

#endif /* INCLUDE_QUATFWD_H_ */

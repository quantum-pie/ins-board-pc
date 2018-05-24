/*
 * IPositionProvider.h
 *
 *      Author: Ermakov_P
 */

#ifndef INCLUDE_IPOSITIONFILTER_H_
#define INCLUDE_IPOSITIONFILTER_H_

#include "core/IFilter.h"
#include "core/IPositionProvider.h"

/*!
 * @brief Position filter interface.
 */
struct IPositionFilter : IFilter, IPositionProvider
{
	/*!
	 * @brief Class destructor.
	 */
	~IPositionFilter() override = default;
};

#endif /* INCLUDE_IPOSITIONFILTER_H_ */

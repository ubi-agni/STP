/**
 * \author Erik Weitnauer
 */
#ifndef _tools_H
#define	_tools_H

#include <iostream>
#include <sstream>
#include <math.h>

#define MIN_VALUE_EQ_ZERO (1.e-8)

/// Returns -1 for x<0, 0 for x==0 and 1 for x>0.
inline int sign(double x) {return (x > 0) - (x < 0);};

/**
 * \brief Returns true if x is close to zero.
 * If the absolute value of the passed argument is less or equal to
 * MIN_VALUE_EQ_ZERO, true is returned.
 */
inline bool isZero(double x) {return fabs(x) < MIN_VALUE_EQ_ZERO; };

/**
 * \brief Returns true if x is close to or greater than zero.
 * If the absolute value of the passed argument is at least 
 *  -(MIN_VALUE_EQ_ZERO), true is returned.
 */
inline bool isPositive(double x) { return x > -MIN_VALUE_EQ_ZERO; };

/**
 * \brief Returns true if x is close to or smaller than zero.
 * If the absolute value of the passed argument is at smaller than 
 *  +(MIN_VALUE_EQ_ZERO), true is returned.
 */
inline bool isNegative(double x) { return x < MIN_VALUE_EQ_ZERO; };

/// write an array of doubles to a stream
void writedArrayToStream(std::ostringstream &oss, const double* a,
                         int start, int end);

#endif	/* _tools_H */


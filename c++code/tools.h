/**
 * \file tools.h
 * \author Erik Weitnauer
 */
#ifndef _tools_H
#define	_tools_H

#include <iostream>
#include <sstream>

using namespace std;

#define MIN_VALUE_EQ_ZERO (1.e-8)

/// Returns -1 for x<0, 0 for x==0 and 1 for x>0.
int sign(double x);

/**
 * \brief Returns true if x is close to zero.
 * If the absolute value of the passed argument is less or equal to
 * MIN_VALUE_EQ_ZERO, true is returned.
 */
bool isZero(double x);

/**
 * \brief Returns true if x is close to or greater than zero.
 * If the absolute value of the passed argument is at least 
 *  -(MIN_VALUE_EQ_ZERO), true is returned.
 */
bool isPositive(double x);

/**
 * \brief Returns true if x is close to or smaller than zero.
 * If the absolute value of the passed argument is at smaller than 
 *  +(MIN_VALUE_EQ_ZERO), true is returned.
 */
bool isNegative(double x);

/// write an array of doubles to a stream
void writedArrayToStream(ostringstream &oss, const double* a,
                         int start, int end);

#endif	/* _tools_H */


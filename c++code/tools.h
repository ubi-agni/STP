// 
// File:   tools.h
// Author: erik
//
// Created on 25. April 2007, 20:36
//

#ifndef _tools_H
#define	_tools_H

#include <iostream>
#include <sstream>

using namespace std;

#define MIN_VALUE_EQ_ZERO (1.e-8)

int sign(double x);

bool isZero(double x);

bool isPositive(double x);

bool isNegative(double x);

void writedArrayToStream(ostringstream &oss, const double* a,
                         int start, int end);

#endif	/* _tools_H */


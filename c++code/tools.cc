/**
 * \file tools.cc
 * \author Erik Weitnauer
 */

#include "tools.h"
#include <sstream>
#include "math.h"

using namespace std;

int sign(double x) {return (x > 0) - (x < 0);};

bool isZero(double x) {return fabs(x) < MIN_VALUE_EQ_ZERO; };

bool isPositive(double x) { return x > -MIN_VALUE_EQ_ZERO; };

bool isNegative(double x) { return x < MIN_VALUE_EQ_ZERO; };


void writedArrayToStream(ostringstream &oss, const double* a,
                         int start, int end) {
    oss << "[";
    for (int i = start; i <= end; i++) {
        if (i>start) oss << ", ";
        oss << a[i];
    }
    oss << "]";
}

/**
 * \file tools.cc
 * \author Erik Weitnauer
 */

#include "tools.hh"

using namespace std;

void writedArrayToStream(ostream &oss, const double* a,
                         int start, int end) {
    oss << "[";
    for (int i = start; i <= end; i++) {
        if (i>start) oss << ", ";
        oss << a[i];
    }
    oss << "]";
}

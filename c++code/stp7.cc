// 
// File:   stp7.cc
// Author: erik
//
// Created on 22. April 2007, 17:06
//

#include "polynomial.h"
#include "complex.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

using namespace std;
//
// 
//
int
main(int argc, char** argv) {
    double coeffs[] = {1,-0.5,-1, -2};
    Polynomial p(3,coeffs);
    cout << "Roots of " << p << ": " << endl;
    cout << p.getRoot(0) << ", "  << p.getRoot(1) << endl;
    printf("Hello World\n");
    return (EXIT_SUCCESS);
}


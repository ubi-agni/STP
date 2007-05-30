// 
// File:   polynomial.h
// Author: erik
//
// Description: A polynomial class for handling polynomials.
// Created on 22. April 2007, 22:52
//

#ifndef _polynomial_H
#define	_polynomial_H

#include "complex.h"
#include <iostream>
#include <string>

class Polynomial {
private:
    int degree;
    Complex* coeff;
    Complex* roots;
    bool foundRoots;
   
protected:
    
public:
    Polynomial(int degree, Complex coeff[]): foundRoots(false) {
        while (coeff[degree].abs() == 0) degree--;
        this->degree = degree;
        this->coeff = new Complex[degree+1];
        for (int i=0; i<=degree; i++) this->coeff[i] = coeff[i];
    };
    
    Polynomial(int degree, double coeff[]): foundRoots(false) {
        while (coeff[degree] == 0) degree--;
        this->degree = degree;
        this->coeff = new Complex[degree+1];
        for (int i=0; i<=degree; i++) {
            this->coeff[i].r = coeff[i];
            this->coeff[i].i = 0.0;
        }
    };
    
    ~Polynomial() {
        delete(coeff);
        delete(roots);
    }
    
    static void laguer(Complex a[], int m, Complex &x, int *its);
    static void findRoots(Complex a[], int m, Complex roots[], bool polish);
    
    Complex getRoot(int i); 
    Complex getCoeff(int i);
    double value(Complex x);
    
    int getDegree() { return degree;};
    
    double getSmallestPositiveRealRoot();
    double getSmallestRealRoot();
    
    std::string toString() const;
    
private:
    void mayComputeRoots();
};

std::ostream& operator<<(std::ostream& os, const Polynomial& p); 

#endif	/* _polynomial_H */


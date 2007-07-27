/**
 * \file polynomial.h
 * \author Erik Weitnauer
 */

#ifndef _polynomial_H
#define	_polynomial_H

#include "complex.h"
#include <iostream>
#include <string>

/**
 * \brief Class for holding and root finding of polynomials.
 * \author Erik Weitnauer
 * \date 2007
 *
 * Using the laguer algorithm decribed in "Numerical Recipes In C", 2nd edition
 * by Press, Teukolsky, e.a. for finding the roots of a (complex) polynomial,
 * p.371ff.
 */
class Polynomial {
private:
    int degree;
    Complex* coeff;
    Complex* roots;
    bool foundRoots;
   
protected:
    
public:
    /**
     * Construcor for complex coefficients.
     * @param[in] degree Degree of the polynom, e.g. 2 for \f$x^2-x+1\f$
     * @param[in] coeff  Array containing the complex coeffs [0..deg]
     *
     * In the case the main coefficient(s) are zero, they are truncated.
     */
    Polynomial(int degree, Complex coeff[]): foundRoots(false) {
        while (coeff[degree].abs() == 0) degree--;
        this->degree = degree;
        this->coeff = new Complex[degree+1];
        for (int i=0; i<=degree; i++) this->coeff[i] = coeff[i];
    };
    /**
     * Construcor for real coefficients.
     * @param[in] degree Degree of the polynom, e.g. 2 for \f$x^2-x+1\f$
     * @param[in] coeff  Array containing the complex coeffs [0..deg]
     *
     * In the case the main coefficient(s) are zero, they are truncated. 
     */
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
    
    /// Implementation of laguer algorithm for finding one root of a given polynominal.
    static void laguer(Complex a[], int m, Complex &x, int *its);
    
    /// Searches for all (complex) roots of a given polynomial.
    static void findRoots(Complex a[], int m, Complex roots[], bool polish);
    
    Complex getRoot(int i); ///< gives back the root with index i
    Complex getCoeff(int i); ///< gives back the coefficient with index i
    Complex value(Complex x); ///< calculates the value of the polynomial at x
    
    int getDegree() { return degree;}; ///< returns the degree
    
     /// get the smallest positive root with no imgaginary part
    double getSmallestPositiveRealRoot();
    /// get the real root that is closest to zero
    double getSmallestRealRoot();
    
    std::string toString() const; ///< convert polynomial to string
    
private:
    void mayComputeRoots(); ///< finds the roots, if they haven't been found before
};

std::ostream& operator<<(std::ostream& os, const Polynomial& p); 

#endif	/* _polynomial_H */


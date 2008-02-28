/**
 * \file polynomial.cc
 * \author Erik Weitnauer
 */

#include <math.h>
#include <iostream>
#include <sstream>
#include <stdexcept>

#include "polynomial.hh"
#include "complex.hh"

/*** DEFINES for Polynomial::findRoots ***/
// fraction for rounding off imaginary part to zero
#define EPS 2.0e-10

/*** DEFINES for Polynomial::laguer ***/
// estimated fractional roundoff error
#define EPSS 1.0e-15
// try to break (rare) limit cycles with...
#define MR 10
// ...different fractional values, once every...
#define MT 10
// ...steps for...
#define MAXIT (MT*MR)
// ...total allowed iterations.
#define FMAX 

using namespace std;

/**
 * .
 * @param[in] a Complex[0..m] of coefficients
 * @param[in] m degree of polynomial
 * @param[out] x the root that was found
 * @param[out] its number of iterations needed
 */
void Polynomial::laguer(Complex a[], int m, Complex &x, int *its) {
    int iter,j;
    double abx,abp,abm,err;
    Complex dx,x1,b,d,f,g,h,sq,gp,gm,g2;
    // fractions to break a limit circle:
    static double frac[] = {0.1,0.5,0.25,0.75,0.13,0.38,0.62,0.88,1.0};
    
    for (iter=1;iter<MAXIT;iter++) {
        *its = iter;
        b = a[m];
        err = b.abs();
        d = Complex(0,0);
        f = Complex(0,0);
        abx = x.abs();
        // efficient computation of the polynomial and its first two derivatives
        for (j=m-1;j>=0;j--) {
            f = x * f + d;
            d = x * d + b;
            b = x * b + a[j];
            err = b.abs() + abx*err;
        }
        err *= EPSS;
        // estimate of roundoff error in evaluating polynomial.
        if (b.abs() <= err) return;     // we are on the root
        g = d/b;                        // the generic case:
        g2 = g*g;                       // use Laguerre's formular
        h = g2 - (f/b)*2;
        sq = (((h*(double)m)-g2)*(double)(m-1)).csqrt();
        gp = g + sq;
        gm = g - sq;
        abp = gp.abs();
        abm = gm.abs();
        if (abp < abm) gp = gm;
        dx = ((abp > 0.0 || abm > 0.0) ? Complex((double)m, 0.0) / gp 
                : Complex(cos(iter), sin(iter))*exp(log(1+abx)));
        x1 = x-dx;
        if (x.r == x1.r && x.i == x1.i) return;     // converged
        if (iter % MT) { x.r = x1.r; x.i = x1.i; }
        else { Complex z = x-dx*frac[iter/MT]; x.r = z.r; x.i = z.i; }
        // every so ofter we take a fractional step to break any limit circle.
    }
    // too many iterations - very unlikely. Try to start with different starting
    // guess for the root.
    throw x;    
}

/**
 * .
 * @param[in] a  Complex[0..m] of coefficients
 * @param[in] m  degree of polynomial
 * @param[out] roots Complex[0..m] all roots that were found
 * @param[in] polish option to further improve the precision of solution,
 * could be necessary for polynomials of very high degree.
 */
void Polynomial::findRoots(Complex a[], int m, Complex roots[], bool polish) {
    int i,its,j,jj;
    Complex x,b,c;
    Complex* ad = new Complex[m+1];
        
    for (j=0; j<=m; j++) (ad[j]) = a[j];  // copy coefficients for deflation
    for (j=m; j>=1; j--) {              // loop over each root to be found
        x = Complex(0,0);               // start root search at zero...
        laguer(ad, j, x, &its);        // ...and find the root
        if (fabs(x.i) <= 2.0*EPS*fabs(x.r)) x.i = 0.0;
        roots[j-1] = x;
        b = ad[j];
        for (jj=j-1; jj>=0; jj--) {     // forward deflation.
            c = ad[jj];
            ad[jj] = b;
            b = x*b + c;
        }
    }
    if (polish) {
        for (j=1; j<=m; j++)            // Polish the roots using undeflated
            laguer(a,m,roots[j-1],&its); // coefficients
    }
    for (j=2; j<=m; j++) {              // Sort roots by their real parts by
        x = roots[j-1];                   // staight insertion. All roots without
        for (i=j-1; i>=1; i--) {        // imaginary part come first.
            if ((roots[i-1].r <= x.r) && 
                ((roots[i-1].i == 0.0) || (x.i != 0.0))) break;
            roots[i+1-1]=roots[i-1];
        }
        roots[i+1-1]=x;        
    }    
}

/**
 * .
 * @param[in] i index
 * @return coefficient
 */
Complex Polynomial::getCoeff(int i) {
    if ((i < 0) && (i > degree)) throw i;
    return coeff[i];
}

void Polynomial::mayComputeRoots() {
    if (!foundRoots) {
        roots = new Complex[degree];
        findRoots(coeff, degree, roots, true);
        foundRoots = true;
    }
}

/**
 * .
 * @param[in] i index
 * @return root
 *
 * Searches for the roots first, if neccessary.
 */
Complex Polynomial::getRoot(int i) {
    if ((i < 0) && (i >= degree)) throw i;
    mayComputeRoots();
    return roots[i];
}

double Polynomial::getSmallestPositiveRealRoot() {
    mayComputeRoots();
    bool found = false;
    int mini;
    for (int i = 0; i < degree; i++) {
        if (roots[i].r > 0 && roots[i].i == 0) {
            if (!found) { mini = i; found = true;}
            else if (roots[i].r < roots[mini].r) mini = i;
        }
    }
    if (!found) throw range_error("No positive real root!");
    return roots[mini].r;
}

double Polynomial::getSmallestRealRoot() {
    mayComputeRoots();
    bool found = false;
    int mini;
    for (int i = 0; i < degree; i++) {
        if (roots[i].i == 0) {
            if (!found) { mini = i; found = true;}
            else if (fabs(roots[i].r) < fabs(roots[mini].r)) mini = i;
        }
    }
    if (!found) throw range_error("No real root!");
    return roots[mini].r;
}

Complex Polynomial::value(Complex x) {
    Complex result = coeff[degree];
    for (int i = degree-1; i >= 0; i--) {
        result = result*x + coeff[i];
    }
    return result;
}

std::string Polynomial::toString() const {
    ostringstream oss;
    for (int i=degree; i>=0; i--) {
        oss << coeff[i] << "*x^" << i;
        if (i>0) oss << " + ";
    }
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Polynomial& p) { 
    os << p.toString(); 
    return os; 
}

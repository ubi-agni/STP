// 
// File:   Complex.cc
// Author: erik
//
// Created on 23. April 2007, 00:13
//
#include "complex.h"
#include <math.h>
#include <iostream>
#include <sstream>

using namespace std;

Complex Complex::operator+(Complex b) {
    return Complex(r+b.r, i+b.i);
}

Complex Complex::operator-(Complex b) {
    return Complex(r-b.r, i-b.i);
}

Complex Complex::operator*(Complex b) {
    return Complex(r*b.r-i*b.i, r*b.i+i*b.r);
}

Complex Complex::operator * (double x) {
    return Complex(r*x, i*x);
}

Complex Complex::operator/(Complex b) {
    Complex c;
    double x, den;
    if (fabs(b.r) >= fabs(b.i)) {
        x = b.i/b.r;
        den = b.r+x*b.i;
        c.r = (r+x*i)/den;
        c.i = (i-x*r)/den;
    } else {
        x = b.r/b.i;
        den = b.i+x*b.r;
        c.r = (r*x+i)/den;
        c.i = (i*x-r)/den;
    }
    return c;
}

Complex Complex::operator = (const Complex b) {
    r = b.r;
    i = b.i;
    return *this;
}

Complex Complex::conjg() {
    return Complex(r,-i);
}

Complex Complex::csqrt() {
    Complex c;
    double x,y,w,v;
    if ((r == 0) && (i == 0)) {
        c.r = 0.;
        c.i = 0.;
    } else {
        x = fabs(r);
        y = fabs(i);
        if (x >= y) {
            v = y/x;
            w = sqrt(x) * sqrt(0.5*(1.0+sqrt(1.0+v*v)));
        } else {
            v = x/y;
            w = sqrt(y) * sqrt(0.5*(v+sqrt(1.0+v*v)));
        }
        if (r >= 0.0) {
            c.r = w;
            c.i = i/(2.0*w);
        } else {
            c.i = (i>=0) ? w : -w;
            c.r = i/(2.0*c.i);
        }
    }
    return c;
}

double Complex::abs() {
    double x,y,ans,temp;
    x = fabs(r);
    y = fabs(i);
    if (x == 0) ans = y;
    else if (y == 0) ans = x;
    else if (x > y) {
        temp = y/x;
        ans = x*sqrt(1.0+temp*temp);
    } else {
        temp = x/y;
        ans = y*sqrt(1.0+temp*temp);
    }        
    return ans;
}

std::string Complex::toString() const {
    std::ostringstream oss;
    oss << "(" << r << ", " << i << ")";
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Complex& c) { 
    os << c.toString(); 
    return os; 
}

// 
// File:   Complex.h
// Author: erik
//
// Created on 23. April 2007, 00:13
//

#ifndef _Complex_H
#define	_Complex_H

#include <iostream>
#include <string>

class Complex {
public:
    double r,i;
    
    Complex() {r = 0; i = 0;};
    Complex(double real, double img=0): r(real), i(img) {};
    
    Complex conjg();
    Complex csqrt();
    double abs();
    
    
    Complex operator + (Complex);        
    Complex operator - (Complex);
    Complex operator * (Complex);
    Complex operator * (double);
    Complex operator / (Complex);
    Complex operator = (const Complex);
    
    std::string toString() const;
};

std::ostream& operator<<(std::ostream& os, const Complex& c);

#endif	/* _Complex_H */


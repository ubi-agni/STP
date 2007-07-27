/**
 * \file complex.h
 * \author Erik Weitnauer
 */

#ifndef _Complex_H
#define	_Complex_H

#include <iostream>
#include <string>

/** 
 * \brief Represents complex numbers and offers basic operators for them.
 * \author Erik Weitnauer
 * \date 2007
 */
class Complex {
public:
    double r; ///< Real part.
    double i; ///< Imaginary part.
    
    Complex() {r = 0; i = 0;};
    Complex(double real, double img=0): r(real), i(img) {};
    
    Complex conjg(); ///< \return conjugate-complex of the number
    Complex csqrt(); ///< \return square root of the number
    double abs(); ///< \return absolute value of the number
    
    
    Complex operator + (Complex); ///< operator for adding complex numbers
    Complex operator - (Complex); ///< operator for subtracting complex numbers
    Complex operator * (Complex); ///< operator for multiplying complex numbers
    Complex operator * (double); ///< operator for scalar multiplication
    Complex operator / (Complex); ///< operator for dividing complex numbers
    Complex operator = (const Complex); ///< assigment operator, copies the value
    
    std::string toString() const; ///< convert number to string
};

std::ostream& operator<<(std::ostream& os, const Complex& c);

#endif	/* _Complex_H */


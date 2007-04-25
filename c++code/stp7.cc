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
#include "stp7.h"
#include <stdexcept>
#include <sstream>

using namespace std;

void Stp7::planProfile() {
    // TODO: write the real thing...
    // now for testing:
    for (int i = 1; i < 8; i++) {
        _t[i] = 1;
        _j[i] = (i>1 && i<7) ? -(i%2) : (i%2);
    }
    _bIsddec = false;
    _bHasCruise = true;
    _sProfileType = Stp7::PROFILE_TT;

}

double Stp7::planFastestProfile(double x0, double xtarget, double v0,
                                double vmax, double a0, double amax,
                                double jmax) {
    // first set object fields
    _vmax = vmax; _amax = amax; _jmax = jmax;
    _x[0] = x0; _x[7] = xtarget;
    _v[0] = v0; _a[0] = a0; _j[0] = 0; _t[0] = 0;
    
    // Do the planning algorithm --> we get back the jerks and time intervalls,
    // so we will need to do some conversion afterwards.
    planProfile();
    
    for (int i = 1; i < 8; i++) {
        // calc x,v,a values for next switch point
        calcjTrack(_t[i], _x[i-1], _v[i-1], _a[i-1], _j[i], _x[i], _v[i], _a[i]);
        cout << "(" << i << ") x=" << (double)_x[i] << ", v=" << _v[i] << ", a=" << _a[i] << endl;
        // convert: time intervalls --> time points
        _t[i] += _t[i-1];
    }
    
    // test, wether algorithm is correct
    //if (_x[7] != xtarget) 
    //    throw logic_error("The planned profile does not reach the goal!");
    
    _plannedProfile = true;
}

bool Stp7::isDoubleDecProfile() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    return _bIsddec;
} 

bool Stp7::hasCruisingPhase() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    return _bHasCruise;
}

string Stp7::getProfileType() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    return _sProfileType;
}

double Stp7::getSwitchTime(int i) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (i<0 || i>7)
        throw out_of_range("Index for time must be in {0,...,7}!");
    return _t[i];
}

double Stp7::getTimeIntervall(int i) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (i<=0 || i>7)
        throw out_of_range("Index for time must be in {1,...,7}!");
    return _t[i]-_t[i-1];
}

double* Stp7::getJerkArray() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    double* j = new double[8];
    for (int i = 0; i < 7; i++) j[i] = _j[i];
    return j;
}

double* Stp7::getTimeArray() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    double* t = new double[8];
    for (int i = 0; i < 7; i++) t[i] = _t[i];
    return t;
}

int Stp7::getPhaseIndex(double t) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (t < 0)
        throw invalid_argument("Negative time.");
    if (t < _t[1]) return 0;
    if (t < _t[2]) return 1;
    if (t < _t[3]) return 2;
    if (t < _t[4]) return 3;
    if (t < _t[5]) return 4;
    if (t < _t[6]) return 5;
    if (t < _t[7]) return 6;
    return 7;
}

void Stp7::calcjTrack(double dt, double x0, double v0, double a0, double j,
                        double &newx, double &newv, double &newa) const {
    double dt2 = dt*dt;
    newx = x0 + v0*dt + (1./2.)*a0*dt2 + (1./6.)*j*dt2*dt;
    newv = v0 + a0*dt + (1./2.)*j*dt2;
    newa = a0 + j*dt;
}

void Stp7::move(double t, double &x, double &v, double &a, double &j) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (t < 0)
        throw invalid_argument("Negative time.");
    int i = getPhaseIndex(t);
    if (i==7) {
        x = _x[7];
        v = a = j = 0;
    } else {
        calcjTrack(t-_t[i], _x[i], _v[i], _a[i], _j[i+1], x, v, a);
        j = _j[i+1];
    }
}

double Stp7::pos(double t) const {
    double x, v, a, j;
    move(t,x,v,a,j);
    return x;
}

double Stp7::vel(double t) const {
    double x, v, a, j;
    move(t,x,v,a,j);
    return v;
}

double Stp7::acc(double t) const {
    double x, v, a, j;
    move(t,x,v,a,j);
    return a;
}

double Stp7::jer(double t) const {
    double x, v, a, j;
    move(t,x,v,a,j);
    return j;
}

void writedArrayToStream(ostringstream &oss, const double* a,
                         int start, int end) {
    oss << "[";
    for (int i = start; i <= end; i++) {
        if (i>start) oss << ", ";
        oss << a[i];
    }
    oss << "]";
}

std::string Stp7::toString() const {
    std::ostringstream oss;
    if (_plannedProfile) {
        if (_bIsddec) oss << "double decceleration ";
        else oss << "canonical ";
        oss << getProfileType() << " ";
        if (_bHasCruise) oss << "with ";
        else oss << " without ";
        oss << "cruising phase (t=";
        writedArrayToStream(oss, _t, 1,7);
        oss << ", j=";
        writedArrayToStream(oss, _j, 1,7);
        oss << ")";
    } else {
        oss << "unplanned profile";
    }
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Stp7& c) { 
    os << c.toString(); 
    return os; 
}

//
// 
//
int
main(int argc, char** argv) {
    double coeffs[] = {1,-0.5,-1, -2};
    Polynomial p(3,coeffs);
    cout << "Roots of " << p << ": " << endl;
    cout << p.getRoot(0) << ", "  << p.getRoot(1) << endl;
    Stp7 stp;
    cout << stp << endl;
    try {
        stp.planFastestProfile(0,10,0,6,0,4,2);
    } catch (invalid_argument &ea) {
        cout << ea.what() << endl;
    } catch (logic_error &el) {
        cout << el.what() << endl;
    }
    cout << stp << endl;
    cout << "p at t = 1.5: " << stp.pos(1.5) << endl;
    
    printf("Hello World\n");
    return (EXIT_SUCCESS);
}


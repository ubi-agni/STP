// 
// File:   stp3.cc
// Author: erik
//
// Created on 25. April 2007, 19:23
//

#include "stp3.h"
#include <sstream>
#include <iostream>
#include <stdexcept>
#include "math.h"
#include "tools.h"

using namespace std;

const string Stp3::PROFILE_STOP = "profile stop";
const string Stp3::PROFILE_T = "T profile";
const string Stp3::PROFILE_W = "W profile";

bool Stp3::isDoubleDecProfile() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    return _bIsddec;
} 

bool Stp3::isTrapezoid() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    return (_t[2] != _t[1]);
}

string Stp3::getProfileType() const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    return _sProfileType;
}

double Stp3::getSwitchTime(int i) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (i<0 || i>3)
        throw out_of_range("Index for time must be in {0,...,3}!");
    return _t[i];
}

double Stp3::getTimeIntervall(int i) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (i<=0 || i>3)
        throw out_of_range("Index for time must be in {1,...,3}!");
    return _t[i]-_t[i-1];
}

void Stp3::getAccArray(double a[4]) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    for (int i = 0; i < 4; i++) a[i] = _a[i];
}

void Stp3::getTimeArray(double t[4]) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    for (int i = 0; i < 4; i++) t[i] = _t[i];
}

void Stp3::getTimeIntArray(double t[4]) const {
    t[0] = 0; t[1] = _t[1]; t[2] = _t[2] - t[1];
    t[3] = _t[3] - _t[2]; t[4] = _t[4] - _t[3];
}

int Stp3::getPhaseIndex(double t) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (t < 0)
        throw invalid_argument("Negative time.");
    if (t < _t[1]) return 0;
    if (t < _t[2]) return 1;
    if (t < _t[3]) return 2;
    return 3;
}

void Stp3::calcaTrack(double dt, double x0, double v0, double a,
                        double &newx, double &newv) const {
    newx = x0 + v0*dt + 0.5*a*dt*dt;
    newv = v0 + a*dt;
}

void Stp3::move(double t, double &x, double &v, double &a) const {
    if (!_plannedProfile) 
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    if (t < 0)
        throw invalid_argument("Negative time.");
    int i = getPhaseIndex(t);
    if (i==3) {
        x = _x[3];
        v = a = 0;
    } else {
        calcaTrack(t-_t[i], _x[i], _v[i], _a[i+1], x, v);
        a = _a[i+1];
    }
}

double Stp3::pos(double t) const {
    double x, v, a;
    move(t,x,v,a);
    return x;
}

double Stp3::vel(double t) const {
    double x, v, a;
    move(t,x,v,a);
    return v;
}

double Stp3::acc(double t) const {
    double x, v, a;
    move(t,x,v,a);
    return a;
}

double Stp3::planFastestProfile(double x0, double xtarget, double v0,
                                double vmax, double amax) {
    // first set object fields
    _vmax = vmax; _amax = amax;
    _x[0] = x0; _x[3] = xtarget;
    _v[0] = v0; _a[0] = 0; _t[0] = 0;
    
    // Do the planning algorithm --> we get back the jerks and time points.
    planProfile();
    
    // calculate the missing x and v values
    for (int i = 1; i < 4; i++) {
        // calc x,v values for next switch point
        calcaTrack(_t[i]-_t[i-1], _x[i-1], _v[i-1], _a[i], _x[i], _v[i]);
    }
    
    // test, wether algorithm is correct
    //if (_x[3] != xtarget) 
    //    throw logic_error("The planned profile does not reach the goal!");
    
    _plannedProfile = true;
    
    return _t[3];
}

void Stp3::planProfile()
{
    double dir, stop, deltaP, deltaT, w;
    double target = _x[3];
    
    _a[2] = 0;
    _bIsddec = false;
    
    /* compute time needed for full stop */
    dir = -sign(_v[0]); // direction of acceleration to stop
    stop = fabs(_v[0]) / _amax;
    
    /* compute final position after full stop */
    stop = _x[0] + stop * (_v[0] + dir * _amax/2. * stop);
    
    if (target == stop) { // after full stop, we are already at the goal
        _t[1] = _t[2] = 0;       // no acceleration, no cruising phase
        _t[3] = fabs(_v[0]) / _amax;
        _a[1] = -dir*_amax; _a[3] = dir*_amax;
        return;
    } else {
        /* direction of cruising phase */
        dir = sign(target-stop);
        
        /* (typical) direction of acceleration / deceleration */
        _a[1] = dir * _amax; _a[3] = -dir * _amax;
        
        /* time to reach cruising speed dir * _vmax (clipping to zero?) */
        _t[1] = (dir * _vmax - _v[0]) / _a[1];
        if (_t[1] < 0) {
            // deceleration to lower max speed than current speed needed
            _a[1] = -_a[1];
            _t[1]  = -_t[1];
            _bIsddec = true; 
        }
        
        /* time to stop from cruising */
        _t[2] = _vmax / _amax;

        /* pos change from acceleration and deceleration only: */
        deltaP  = _t[1] * (_v[0] + _a[1]/2. * _t[1]);
        deltaP += _t[2] * (dir * _vmax + _a[3]/2. * _t[2]);

        /* time in cruising phase: */
        deltaT = (target - _x[0] - deltaP) / (dir * _vmax);
        if (deltaT >= 0.0) { // plan a complete (trapezoidal) profile:
             _t[3] = _t[1] + deltaT + _t[2]; // duration
            _t[2] = _t[3] - _t[2];
        } else { // plan an incomplete (triangular) profile:
         /* w - speed at switching between acceleration and deceleration */
         w = dir * sqrt (dir * _amax * (target-_x[0]) + _v[0]*_v[0]/2.);
         _t[1] = (w - _v[0]) / _a[1];
         _t[2] = _t[1];
         _t[3] = _t[1] + fabs (w / _a[3]); // duration
        }
    }
}

//bool Stp3::scaleToDuration (double dNewDuration) {
//   double A, B, tcruise, deltaT, diff, stopT, stop, dir;
//   if (dNewDuration <= t3) return false; /* only enlarge duration */
//
//   tcruise = t2-t1;  // old cruising time
//   A = fabs(c[1]) * (dNewDuration - t3) / aMax;
//   B = dNewDuration - t3 + tcruise;
//   /* compute time delta to steel from acc + decl phase */
//   deltaT = -B/2. + sqrt (B*B/4. + A); /* > 0 */
//
//   if (!bDoubleDeceleration && t1 - deltaT >= 0) {
//      t1 += -deltaT;
//      diff = t3 - t2;
//      t3 = dNewDuration;
//      t2 = t3 - diff + deltaT;
//
//      c[1] = a[1] + 2. * a[2] * t1;
//   } else {
//      /* compute time needed for full stop */
//      dir = -sign (a[1]); // direction of acceleration to stop
//      stopT = fabs(a[1] / aMax);
//      /* compute final position after full stop */
//      stop = a[0] + stopT * (a[1] + dir * aMax/2. * stopT);
//
//      /* cruising speed: */
//      c[1] = (goal - stop) / (dNewDuration - stopT);
//      /* turn acceleration into deceleration: */
//      if (!bDoubleDeceleration) {
//         a[2] = -a[2];
//         bDoubleDeceleration = true;
//      }
//      /* time to reach cruising speed: */
//      t1 = fabs(c[1] - a[1]) / aMax;
//      t2 = dNewDuration - (stopT - t1);
//      t3 = dNewDuration;
//   }
//
//   c[0] = parabel (t1, a) - c[1] * t1;
//
//   d[1] = c[1] - 2. * d[2] * t2;
//   d[0] = c[0] + t2 * c[1] - t2 * (d[1] + t2 * d[2]);
//
//   return true;
//}

std::string Stp3::toString() const {
    std::ostringstream oss;
    if (_plannedProfile) {
        if (_bIsddec) oss << "double decceleration ";
        else oss << "canonical ";
        oss << getProfileType() << " (t=";
        writedArrayToStream(oss, _t, 1,3);
        oss << ", a=";
        writedArrayToStream(oss, _a, 1,3);
        oss << ")";
    } else {
        oss << "unplanned profile";
    }
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Stp3& c) { 
    os << c.toString(); 
    return os; 
}

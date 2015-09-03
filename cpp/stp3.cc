/**
 * \file stp3.cc
 * \author Erik Weitnauer
 */

#include <sstream>
#include <iostream>

#include "stp3.hh"
#include "tools.hh"

using namespace std;

const string Stp3::PROFILE_STOP = "profile stop";
const string Stp3::PROFILE_T = "T profile";
const string Stp3::PROFILE_W = "W profile";

bool Stp3::isDoubleDecProfile() const {
    return _bIsddec;
} 

/**
 * .
 * \throws invalid_argument if planFastesProfile(..) wasn't called before
 */
bool Stp3::isTrapezoid() const {
    return (_t[2] != _t[1]);
}

/**
 * .
 * \throws invalid_argument if planFastesProfile(..) wasn't called before
 */
string Stp3::getProfileType() const {
    return _sProfileType;
}

/**
 * .
 * @param[in] i index, 1 <= i <= 3
 * \throws out_of_range if i is out of range
 */
double Stp3::getSwitchTime(int i) const {
    if (i<0 || i>3)
        throw out_of_range("Index for time must be in {0,...,3}!");
    return _t[i];
}

/**
 * .
 * \param[in] i index, 1 <= i <= 3
 * \throws out_of_range if i is out of range
 */
double Stp3::getTimeIntervall(int i) const {
    if (i<=0 || i>3)
        throw out_of_range("Index for time must be in {1,...,3}!");
    return _t[i]-_t[i-1];
}

void Stp3::getAccArray(double a[4]) const {
    for (int i = 0; i < 4; i++) a[i] = _a[i];
}

void Stp3::getTimeArray(double t[4]) const {
    for (int i = 0; i < 4; i++) t[i] = _t[i];
}

void Stp3::getTimeIntArray(double t[4]) const {
    t[0] = 0; t[1] = _t[1]; t[2] = _t[2] - t[1];
    t[3] = _t[3] - _t[2]; t[4] = _t[4] - _t[3];
}

bool Stp3::isValidMovement() const {
	if (_t[0] < 0) return false;
	for (int i = 1; i < 4; i++) if (_t[i] < _t[i-1]) return false;
	for (int i = 1; i < 4; i++) if (fabs(_a[i]) > _amax) return false;
	return true;
}

/**
 * .
 * For t = 0 returns 1, for t >= duration returns 3.
 */
int Stp3::getPhaseIndex(double t) const {
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

/**
 * .
 * @throws logic_error if no solution was found
 * @return duration of planned trajectory
 */
double Stp3::planFastestProfile(double x0, double xtarget, double v0,
                                double vmax, double amax) throw(logic_error) {
	// check, whether vmax and amax are greater than zero
	if (isNegative(vmax) || isNegative(amax))
		throw invalid_argument("vmax and amax must be positive!");
	
	// first set object fields
    _vmax = vmax; _amax = amax;
    _x[0] = x0; _x[3] = xtarget;
    _v[0] = v0; _a[0] = 0; _t[0] = 0;
    
    // Do the planning algorithm --> we get back the jerks and time points.
    planProfile();
	// check if we have valid times, if not, throw a logic error
	if (!isValidMovement()) throw logic_error("Invalid solution.");

    // calculate the missing x and v values
    for (int i = 1; i < 4; i++) {
        // calc x,v values for next switch point
        calcaTrack(_t[i]-_t[i-1], _x[i-1], _v[i-1], _a[i], _x[i], _v[i]);
    }

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

/**
 * .
 * @throws logic_error if no solution was found
 */
double Stp3::scaleToDuration(double dNewDuration) throw(logic_error) {
    if (!_plannedProfile) 
        return 0;
    
    if (dNewDuration <= _t[3]) return _t[3]; // only enlarge duration
    
    double A, B, tcruise, deltaT, diff, stopT, stop, dir;
    
    tcruise = _t[2] - _t[1];  // old cruising time
	A = fabs(_v[1]) * (dNewDuration - _t[3]) / _amax;
	B = dNewDuration - _t[3] + tcruise;
	
	/* compute time delta to steel from acc + decl phase */
	deltaT = -B/2. + sqrt (B*B/4. + A); /* > 0 */
	
	if (!_bIsddec && (_t[1] - deltaT >= 0)) {
		_t[1] -= deltaT;
        _t[2] = dNewDuration - (_t[3] - _t[2]) + deltaT;
        _t[3] = dNewDuration;
        
        _v[1] = _v[0] + _a[1] * _t[1];
    } else {
        /* compute time needed for full stop */
        dir = -sign (_v[0]); // direction of acceleration to stop
        stopT = fabs(_v[0] / _amax);
        /* compute final position after full stop */
        stop = _x[0] + stopT * (_v[0] + dir * _amax/2. * stopT);

	    /* cruising speed: */
        _v[1] = (_x[3] - stop) / (dNewDuration - stopT);
        /* turn acceleration into deceleration: */
        if (!_bIsddec) {
           _a[1] = -_a[1];
           _bIsddec = true;
        }
        /* time to reach cruising speed: */
        _t[1] = fabs(_v[1] - _v[0]) / _amax;
        _t[2] = dNewDuration - (stopT - _t[1]);
        _t[3] = dNewDuration;
     }
     
     // calculate the missing x and v values
    for (int i = 1; i < 4; i++) {
        // calc x,v values for next switch point
        calcaTrack(_t[i]-_t[i-1], _x[i-1], _v[i-1], _a[i], _x[i], _v[i]);
    }
    
	// check if we have valid times, if not, throw a logic error
	if (!isValidMovement()) throw logic_error("No solution found for stretched 3stp profile.");
	
    return _t[3];
}

std::string Stp3::toString() const {
    std::ostringstream oss;
    if (_plannedProfile) {
        if (_bIsddec) oss << "double decceleration ";
        else oss << "canonical ";
        oss << getProfileType() << " (t=";
        writedArrayToStream(oss, _t, 1,3);
        oss << ", a=";
        writedArrayToStream(oss, _a, 1,3);
		oss << ", x0 = " << _x[0] << ", xTarget = " << _x[3] << ", v0 = ";
		oss << _v[0] << ", vmax = " << _vmax << ", amax = " << _amax << ")";
    } else {
        oss << "unplanned profile";
    }
    return oss.str();
}

double Stp3::getEndOfCruisingTime() const {
	return _t[2];	
}

std::ostream& operator<<(std::ostream& os, const Stp3& c) { 
    os << c.toString(); 
    return os; 
}

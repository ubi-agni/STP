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
#include "tools.h"
#include "stp3.h"
#include "stp7.h"
#include <stdexcept>
#include <sstream>
#include "math.h"
#include <cxxtest/TestSuite.h>

using namespace std;

// initialize Stp7 string constants...
const string Stp7::PROFILE_STOP = "stop profile";
const string Stp7::PROFILE_TT = "TT profile";
const string Stp7::PROFILE_TW = "TW profile";
const string Stp7::PROFILE_WT = "WT profile";
const string Stp7::PROFILE_WW = "WW profile";

void Stp7::planProfile() {
    /* Calculates the time optimal third-order trajectory to reach the target
     * position with the given start conditions and according to the limitations
     * for jerk, acc and vel.
     * There are two arrays filled up with the correct values: The array holding
     * the jerk impulses and the one describing the time intervalls for these
     * impulses. Both have a fixed length of 7 entries, however, several entries
     * in the t-array might be zero - indicating that this paricular phase is
     * not needed in the profil.
     * The elements of the jerk array are the jerk values (NOT: either of the
     * three values -1, 0, 1. NOT: Multiplication with the max-jerk gives the
     * actual jerk-value.)
     */
    
    int dir = 0;
    double xTarget = _x[7];
    double x_dummy, a_dummy, v_dummy;
    double xStop;
    
    // (1) Calculation of the direction flag (direction of potential cruising
    // phase) by comparing the position we reach at an immideate halt to the
    // desired target position.
    Stp3 stp3Stop;
    stp3Stop.planFastestProfile(_v[0], 0, _a[0], _amax, _jmax);
    stp3Stop.getTimeArray(_t);
    stp3Stop.getAccArray(_j);
    
    Stp7::calcjTracks(_t, _j, 3, _x[0], _v[0], _a[0], xStop, v_dummy, a_dummy);
    dir = sign(xTarget-xStop);
    if (dir == 0) {
        for (int i = 4; i < 8; i++) {
            _t[i] = _t[3]; _j[i] = 0;
        }
        _sProfileType = Stp7::PROFILE_STOP;
        _bIsddec = false;
        _bHasCruise = false;
        return;
    } else {
        // position change just from acc and dec phase:
        Stp3 stp3Acc, stp3Dec;
        stp3Acc.planFastestProfile(_v[0], dir*_vmax, _a[0], _amax, _jmax);
        stp3Dec.planFastestProfile(dir*_vmax, 0, 0, _amax, _jmax);
        // position change:
        stp3Acc.getTimeArray(_t);
        stp3Acc.getAccArray(_j);
        stp3Dec.getTimeArray(&(_t[4]));
        stp3Dec.getAccArray(&(_j[4]));
        _t[4] = 0; _j[4] = 0;
        for (int i = 4; i < 8; i++) _t[i] += _t[3];
        Stp7::calcjTracks(_t, _j, 7, _x[0], _v[0], _a[0],
        xStop, v_dummy, a_dummy);
        // distance we need to go in cruising phase:
        double xDelta = (xTarget-xStop);
        double tDelta = xDelta / (dir*_vmax);
        
        // case differentiation: Do we have a cruising phase?
        if (tDelta >= 0) {
            // with cruising phase, insert t_delta as cruising phase (t[4])
            for (int i = 4; i < 8; i++) _t[i] += tDelta;
            _bIsddec = false;
            _bHasCruise = true;
            if (stp3Acc.isTrapezoid())
                _sProfileType = (stp3Dec.isTrapezoid()) ? Stp7::PROFILE_TT
                : Stp7::PROFILE_TW;
            else 
                _sProfileType = (stp3Dec.isTrapezoid()) ? Stp7::PROFILE_WT
                : Stp7::PROFILE_WW;
            _bIsddec = (_j[1]==_j[5]);
        } else {
            // without cruising phase TODO TODO
            planProfileNoCruise(dir);
            _bHasCruise = false;
        }
    }
}

void Stp7::convertTimePointsToIntervalls() {
    for (int i = 7; i > 0; i--) {
        _t[i] = _t[i] - _t[i-1];
    }
}

void Stp7::convertTimeIntervallsToPoints() {
    for (int i = 1; i < 8; i++) {
        _t[i] = _t[i] + _t[i-1];
    }
}

void Stp7::planProfileNoCruise(int dir) {
    // The function must be called with a valid 7-phases profile stored in
    // _t and _j arrays, which is overshooting the target. It will then first do
    // a case distinction to check, which kind of profile we currently have.
    // Next step is to cut out / shift parts of the acc-profile to shorten the
    // profile until it is not overshooting the target anymore. At this time we
    // know which profile-type the final solution will have and can call the
    // appropriate function to find the final solution.

    // (0) Check whether a normal profile has to switch into double deceleration
    // (1) Check whether we have a double deceleration profile.
    // (2) Case distinction: TT / TW / WT / WW
    
    // its easier to calculate in time intervalls than time points, so we
    // convert the time array into intervalls first. In the end we just
    // convert it back.
    _bHasCruise = false;
    convertTimePointsToIntervalls();
    
    double x_dummy, v_dummy, a_dummy;
    
    // (0)
    if (sign(_j[3]) == sign(_j[5]) && _t[3] < _t[1]) {
        double tAcc[] = {0, _t[3],  _t[2], _t[3]};
        double tDec[] = {0, _t[5],  _t[6], _t[7]};
        double deltaAcc, deltaDec;
        Stp7::calcjTracksTimeInt(tAcc, _j, 3, 0, 0, 0, x_dummy, deltaAcc, a_dummy);
        Stp7::calcjTracksTimeInt(tDec, &(_j[4]), 3, 0, 0, 0, x_dummy, deltaDec, a_dummy);
        deltaAcc = fabs(deltaAcc);
        deltaDec = fabs(deltaDec);
        if (deltaAcc < deltaDec) {
            tAcc[1] = tAcc[2] = 0; tAcc[3] = _t[1]-_t[3];
            Stp7::removeAreaTimeInt(tDec, deltaAcc, _amax, _jmax);
            double jNew[] = {0, _j[3], 0, _j[1], 0, _j[5], _j[6], _j[7]};
            double tNew[8];
            for (int i = 0; i < 8; i++) tNew[i] = (i<4) ? tAcc[i] : tDec[i-4];
            // If we still overshoot after putting as much area under the acc
            // graph from the front to the back, the profile becomes double
            // deceleration.
            if (stillOvershootsTimeInt(tNew, jNew, 7, dir, _x[0], _x[7], _v[0], _a[0]))
                for (int i = 0; i < 8; i++) {_t[i] = tNew[i]; _j[i] = jNew[i]; }
        }
    }

    // (1)
    // check if we have a double deceleration profile
    if (sign(_j[3]) != sign (_j[5])) {
        bool bSecondTrapezoidal = false;
        if (_t[6] == 0) {
            // second part is currently wedge, may become trapez
            // calculate maximal shift from first to second deceleration phase
            // in order to reach the W-T-border case
            double a2, v2;
            Stp7::calcjTracksTimeInt(_t, _j, 2, _x[0], _v[0], _a[0], x_dummy, v2, a2);
            double tDelta, t5, t7;
            Stp7::calc7st_opt_shiftTimeInt(_t, _j, dir, _amax, _jmax, v2, a2, tDelta, t5,t7);
            if (tDelta < 0) throw out_of_range("DeltaT negative at opt_shift!");
            if (tDelta < _t[3]) {
                // adapt profile by shortening t[3]
                double tNew[] = {0, _t[1], _t[2], _t[3]-tDelta, _t[4], t5, 0, t7};
                // if we still overshoot, the profile becomes trapezoidal
                if (stillOvershootsTimeInt(tNew, _j, 7, dir, _x[0], _x[7], _v[0], _a[0])) {
                    for (int i = 0; i < 8; i++) _t[i] = tNew[i];
                    // allow trapez in second part when generating formulas:
                    _t[6] = 1;
                    bSecondTrapezoidal = true;
                }
            } else {
                // velocity delta in phase 3 is not enough to extend
                // wedge-shaped second decleration phase to trapezoidal shape
                // so we stay at a triangular profile    
            }  
        }
        
        if (bSecondTrapezoidal) {
            // actually, we only know its a ?-T profile, but for the sake of
            // testing before we finish the solving algorithm we set it to WT...
            _sProfileType = Stp7::PROFILE_WT;
        } else {
            _sProfileType = Stp7::PROFILE_WW; // could also be TW...
        }
        // Calculate exact phase duration from given profile t, j
        
        
        // TODO: call solver for double decleration case!
        // t[1], t[2] and j[0..7] already have their correct values!
        TS_WARN("ddec - TODO");
        
        _sProfileType = getProfileString(_t);
        _bIsddec = true;
        convertTimeIntervallsToPoints();
        return;
    }
    
    // (2)
    // we don't have double deceleration --> cut out instead of merging
    // find correct profile by cutting pieces and descending to shorter profiles
    _bIsddec = false;
    _sProfileType = Stp7::findProfileTimeInt(_t, _j, dir, _x[0], _x[7], _v[0], _a[0],
                                     _amax, _jmax);
    
    // Calculate exact phase duration for choosen profile t, j
    // j[0..7] are already the correct values!
    if (_sProfileType == Stp7::PROFILE_TT) {
        TS_WARN("canonical TT - TODO");
    } else if (_sProfileType == Stp7::PROFILE_TW) {
        TS_WARN("canonical TW - TODO");
    } else if (_sProfileType == Stp7::PROFILE_WT) {
        TS_WARN("canonical WT - TODO");
    } else if (_sProfileType == Stp7::PROFILE_WW) {
        TS_WARN("canonical WW - TODO");
        //solveProfileWW(_t, _x[0], _x[7], _v[0], _a[0], _jmax, dir);
    } else throw invalid_argument("Full stop case should be taken care of earlier!");
    
    convertTimeIntervallsToPoints();
    return;
}

void Stp7::solveProfileWW(double t[8], double x0, double xTarget, double v0, double a0, double j, int dir) {
    double a2=a0*a0;
    double a3 = a2*a0;
    double a4=a3*a0;
    double v2=v0*v0;
    double v3=v0*v2;
    double j2 = j*j;
    double j3 = j2*j;
    double j4 = j3*j;
    double d_c = dir;
    double coeff[5];
    double xTarget2 = xTarget*xTarget;
    coeff[0] = -3.*a4 -12.*v2*j2 +12.*a2*v0*d_c*j;
    coeff[1] = -48.*xTarget*j2 +48.*x0*j2 -48.*a0*v0*d_c*j+ 16.*a3;
    coeff[2] = 48.*v0*d_c*j -24.*a2;
    coeff[3] = 0.;
    coeff[4] = 12.;
    Polynomial poly(4, coeff);
    double R = poly.getSmallestPositiveRealRoot();
    double R2 = R*R;
    t[1] = 0.25*(a2-4.*R*a0+2.*R2-2.*v0*d_c*j)/(R*j*d_c);
    t[2] = 0.;
    t[3] = R/j*d_c;
    t[4] = t[5] = t[6] = 0.;
    t[7] = 0.25*(-a2+2.*R2+2.*v0*d_c*j)/(R*j*d_c);
}

string Stp7::getProfileString(double t[8]) {
    if (t[2] != 0) { // T? profile
        if (t[6] != 0) return Stp7::PROFILE_TT;
        else return Stp7::PROFILE_TW;
    } else { // W? profile
        if (t[6] != 0) return Stp7::PROFILE_WT;
        else return Stp7::PROFILE_WW;
    } 
}

string Stp7::findProfileTimeInt(double t[8], double j[8], int dir, double x0,
                         double xTarget, double v0, double a0, double amax,
                         double jmax) {
    // find correct profile by cutting pieces and descending to shorter profiles
    // uses the values t[1..7] and j[1..7] and changes them accodingly to the
    // new profile type.
    string type = getProfileString(t);
    
    double tOld[8];
    for (int i = 0; i < 8; i++) tOld[i] = t[i];

    if (type == Stp7::PROFILE_TT) {
        // cut out smaller a=const. part
        double dt = min(t[2], t[6]);
        t[2] = t[2] - dt;
        t[6] = t[6] - dt;
        if (stillOvershootsTimeInt(t, j, 7, dir, x0, xTarget, v0, a0)) {
            // recursively calling this function even cuts further
            type = findProfileTimeInt(t,j,dir,x0,xTarget,v0,a0,amax,jmax);
        } else {
            // now we stop before the target, hence profile stays TT
            for (int i = 0; i < 8; i++) t[i] = tOld[i];
        }
        return type;
    }

    if (type == Stp7::PROFILE_WW) {
        // nothing to do, WW stays WW anytime
        return type;
    }
    
    if (type == Stp7::PROFILE_WT) {
        double a1 = a0 + j[1]*t[1];
        double dt_w = min(t[1],t[3]);
        double area_w_max = fabs(dt_w * (2*a1 - dt_w*j[1]));
        double area_t_max = t[6]*amax;
        if (area_w_max > area_t_max) {
            // we will cut out the whole t[6] WT -> WW
            t[6] = 0;
            double dt = (fabs(a1)-sqrt(a1*a1-area_t_max))/jmax;
            t[1] = t[1]-dt;
            t[3] = t[3]-dt;
            if (stillOvershootsTimeInt(t, j, 7, dir, x0, xTarget, v0, a0)) {
                type = Stp7::PROFILE_WW; // type switches to WW
            } else {
                // now we stop before the target, hence profile stays WT
                for (int i = 0; i < 8; i++) t[i] = tOld[i];
            }
        } else; // nothing to cut out, stays WT
        return type;
    }
    
    if (type == Stp7::PROFILE_TW) {
        double a5 = j[5]*t[5];
        double area_w_max = fabs(t[5]*a5);
        double area_t_max = t[2]*amax;
        if (area_w_max > area_t_max) {
            // we will cut out the whole t[2]
            t[2] = 0;
            t[5] = sqrt((area_w_max-area_t_max)/fabs(j[5]));
            t[7] = t[5];
            if (stillOvershootsTimeInt(t, j, 7, dir, x0, xTarget, v0, a0)) {
                type = Stp7::PROFILE_WW;
            } else {
                // now we stop before the target, hence profile stays TW
                for (int i = 0; i < 8; i++) t[i] = tOld[i];
            }
        }
        return type;
    }
}

void Stp7::calc7st_opt_shiftTimeInt(double t[8], double j[8], int dir, double amax,
                                  double jmax, double v2, double a2,
                                  double &tDelta, double &t5, double &t7) {
    // Given a deceleration - deceleration profile with wedge-shaped second part, 
    // compute the period DeltaT which must be cut from third phase (and
    // inserted in second part), such that the second part becomes a triangular
    // profile exactly hitting -d*amax.
    double v3, a3, v6, a6, x_dummy;
    // compute a3 and a6
    Stp7::calcjTrack(t[3], 0., v2, a2, j[3], x_dummy, v3, a3);
    Stp7::calcjTracksTimeInt(&(t[3]), &(j[3]), 3, 0, v3, a3, x_dummy, v6, a6);
    
    // compute discriminant of quadratic polynomial solution
    double diskriminant = 4*amax*amax + 2*jmax*jmax*(t[7]*t[7] - t[5]*t[5])
                          + 4*dir*jmax*(a6*t[7] + a3*t[5]) + 2*a3*a3;
    if (isZero(diskriminant)) diskriminant = 0;
    double root = sqrt(diskriminant);
    
    // compute T5
    if (dir < 0) t5 = dir * amax + root/2;
    else t5 = dir * amax - root/2;
    t5 = t5 / (dir * jmax);

    // compute DeltaT and T7
    tDelta = (a3 + dir * amax) / (dir * jmax) - t5;
    if (isZero(tDelta)) tDelta = 0;
    t7 = amax / jmax;                                 
}

    
void Stp7::removeAreaTimeInt(double t[4], double deltaV, double amax, double jmax) {
    // Takes an array t[1..3] and deletes the passed deltaV from the area under
    // the acceleration graph. The elements of t are altered accordingly.
    
    // we only decrease the area...
    deltaV = fabs(deltaV);
    double A_now = t[1]*t[1]*jmax+amax*t[2];
    if (A_now < deltaV) return;        // not enough to cut out...
    double Aw_max = amax*amax/jmax;
        
    if (isZero(t[2]) || (Aw_max > A_now - deltaV)) {
        // result wedge shaped
        t[1] = sqrt(A_now/jmax - deltaV/jmax);
        t[3] = t[1];
        t[2] = 0;
    } else {
        // result trapezoid shaped
        t[2] = t[2] - deltaV/amax;
    }
}

bool Stp7::stillOvershootsTimeInt(double t[], double j[], int length, int dir,
                            double x0, double xTarget, double v0, double a0) {
    double v_dummy, a_dummy, xEnd;
    Stp7::calcjTracksTimeInt(t, j, length, x0, v0, a0, xEnd, v_dummy, a_dummy);
    return (sign(xEnd - xTarget)*dir == 1);                            
}

double Stp7::planFastestProfile(double x0, double xtarget, double v0,
                                double vmax, double a0, double amax,
                                double jmax) {
    // first set object fields
    _vmax = vmax; _amax = amax; _jmax = jmax;
    _x[0] = x0; _x[7] = xtarget;
    _v[0] = v0; _a[0] = a0; _j[0] = 0; _t[0] = 0;
    
    // Do the planning algorithm --> we get back the jerks and times.
    planProfile();
    
    for (int i = 1; i < 8; i++) {
        // calc x,v,a values for next switch point
        calcjTrack(_t[i]-_t[i-1], _x[i-1], _v[i-1], _a[i-1], _j[i], _x[i], _v[i], _a[i]);
    }
    
    // test, wether algorithm is correct
    //if (_x[7] != xtarget)
    //    throw logic_error("The planned profile does not reach the goal!");
    
    _plannedProfile = true;
    
    return _t[7];
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

void Stp7::calcjTrack(double dt, double x0, double v0, double a0,
double j, double &newx, double &newv, double &newa) {
    double dt2 = dt*dt;
    newx = x0 + v0*dt + (1./2.)*a0*dt2 + (1./6.)*j*dt2*dt;
    newv = v0 + a0*dt + (1./2.)*j*dt2;
    newa = a0 + j*dt;
}

void Stp7::calcjTracks(double t[], double j[], int length, double x0,
double v0, double a0, double &x, double &v, double &a) {
    x = x0; v = v0; a=a0;
    for (int i = 1; i <= length; i++) {
        calcjTrack(t[i]-t[i-1], x, v, a, j[i], x, v, a);
    }
}

void Stp7::calcjTracksTimeInt(double t[], double j[], int length,
             double x0, double v0, double a0, double &x, double &v, double &a) {
    x = x0; v = v0; a=a0;
    for (int i = 1; i <= length; i++) {
        calcjTrack(t[i], x, v, a, j[i], x, v, a);
    }
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
        Stp7::calcjTrack(t-_t[i], _x[i], _v[i], _a[i], _j[i+1], x, v, a);
        j = _j[i+1];
    }
}

double Stp7::pos(double t) const {
    double x, v, a, j;
    move(t, x, v, a, j);
    return x;
}

double Stp7::vel(double t) const {
    double x, v, a, j;
    move(t, x, v, a, j);
    return v;
}

double Stp7::acc(double t) const {
    double x, v, a, j;
    move(t, x, v, a, j);
    return a;
}

double Stp7::jer(double t) const {
    double x, v, a, j;
    move(t, x, v, a, j);
    return j;
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
        writedArrayToStream(oss, _t, 1, 7);
        oss << ", j=";
        writedArrayToStream(oss, _j, 1, 7);
        oss << ", x0 = " << _x[0] << ", xTarget = " << _x[7] << ", v0 = ";
        oss << _v[0] << ", vmax = " << _vmax << ", a0 = " << _amax;
        oss << ", amax = " << _amax << ")";
    } else {
        oss << "unplanned profile";
    }
    return oss.str();
}

std::ostream& operator<<(std::ostream& os, const Stp7& c) {
    os << c.toString();
    return os;
}

////
////
////
//int
//main(int argc, char** argv) {
//    double coeffs[] = {1, -0.5, -1, -2};
//    Polynomial p(3, coeffs);
//    cout << "Roots of " << p << ": " << endl;
//    cout << p.getRoot(0) << ", "  << p.getRoot(1) << endl;
//    return (EXIT_SUCCESS);
//}
//

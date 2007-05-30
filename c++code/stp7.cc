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
        //TS_WARN("ddec - TODO");
        
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
    
    double da;
    da = _j[1] == _j[3] ? -1 : 1;
    // Calculate exact phase duration for choosen profile t, j
    // j[0..7] are already the correct values!
    if (_sProfileType == Stp7::PROFILE_TT) {
        //TS_WARN("canonical TT - TODO");
        solveProfileTT(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
    } else if (_sProfileType == Stp7::PROFILE_TW) {
        //TS_WARN("canonical TW - TODO");
        solveProfileTW(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
    } else if (_sProfileType == Stp7::PROFILE_WT) {
        //TS_WARN("canonical WT - TODO");
        solveProfileWT(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
    } else if (_sProfileType == Stp7::PROFILE_WW) {
        //TS_WARN("canonical WW - TODO");
        solveProfileWW(_t, _x[0], _x[7], _v[0], _a[0], _jmax, da, dir);
    } else throw invalid_argument("Full stop case should be taken care of earlier!");
    
    convertTimeIntervallsToPoints();
    return;
}

void Stp7::solveProfileWW(double t[8], double x0, double xTarget, double v0, double a0, double jmax, double da, double dc) {
    double root;
    double a2 = a0 * a0;
    double jmax2 = jmax * jmax;
    {
        double t2 = a2 * da;
        double t8 = dc * jmax;
        double t14 = a0 * v0;
        double t23 = a2 * a0;
        double t27 = jmax2 * da;
        double t30 = jmax2 * x0;
        double t36 = a2 * a2;
        double t39 = jmax2 * a0;
        double t40 = da * xTarget;
        double t43 = v0 * v0;
        double t51 = da * x0;
        double t65 = jmax2 * jmax;
        double t66 = t65 * v0;
        double t70 = jmax2 * a2;
        double t78 = dc * da;
        double t87 = t36 * a0;
        double t96 = t23 * v0;
        double t104 = -0.144e3 * t66 * dc * xTarget - 0.144e3 * t70 * t40
                      + 0.144e3 * t70 * t51 + 0.144e3 * t66 * dc * x0
                      - 0.144e3 * t66 * t78 * xTarget - 0.72e2 * t39 * t43
                      + 0.144e3 * t66 * t78 * x0 - 0.6e1 * t87 - 0.6e1 * t87 * da
                      - 0.72e2 * t39 * da * t43 - 0.144e3 * t70 * xTarget
                      - 0.24e2 * t8 * t96 - 0.24e2 * t8 * t96 * da + 0.144e3 * t70 * x0;
        double t109 = jmax2 * jmax2;
        double t113 = dc * a0;
        double t127 = xTarget * xTarget;
        double t130 = x0 * x0;
        double coeffs[5];
        coeffs[4] = -0.18e2 * t2 + 0.36e2 * v0 * dc * jmax - 0.18e2 * a2 + 0.36e2 * t8 * v0 * da;
        coeffs[3] = 0.72e2 * t14 * t8 + 0.72e2 * t8 * t14 * da - 0.72e2 * jmax2 * xTarget
                    - 0.48e2 * t23 - 0.48e2 * t23 * da - 0.72e2 * t27 * xTarget
                    + 0.72e2 * t30 + 0.72e2 * t30 * da;
        coeffs[2] = -0.27e2 * t36 * da - 0.216e3 * t39 * t40 + 0.36e2 * t27 * t43
                    + 0.216e3 * t39 * x0 - 0.36e2 * t8 * t2 * v0 + 0.216e3 * t39 * t51
                    - 0.36e2 * jmax * v0 * dc * a2 - 0.216e3 * t39 * xTarget
                    - 0.27e2 * t36 + 0.36e2 * t43 * jmax2;
        coeffs[1] = t104;
        coeffs[0] = - 0.6e1 * t36 * v0 * t8 - 0.144e3 * x0 * t109 * xTarget
                    + 0.144e3 * t66 * t113 * x0 - 0.144e3 * t66 * t113 * xTarget
                    - 0.72e2 * t65 * t43 * v0 * dc - 0.48e2 * t23 * xTarget * jmax2
                    + 0.72e2 * t127 * t109 + 0.72e2 * t130 * t109 - 0.36e2 * a2 * t43 * jmax2
                    + 0.48e2 * t23 * x0 * jmax2 - t36 * a2;
        Polynomial p(4, coeffs);
        root = p.getSmallestRealRoot();
    }
    t[1] = root / dc / jmax;
    t[2] = 0;
    double root2 = root*root;
    {
        double t22 = 0.3e1 * root2 * jmax;
        double t25 = 0.6e1 * jmax * a0 * root;
        t[3] = (a2 * a0 + 0.3e1 * a2 * da * root - 0.6e1 * jmax2 * x0 - 0.6e1 * root * dc * jmax * v0 + 0.6e1 * jmax2 * xTarget)
                / (0.6e1 * jmax2 * v0 + (0.3e1 * jmax * a2 + t22 + t25) * dc + (t25 + t22) * dc * da);
    }
    t[4] = 0;
    t[5] = 0;
    t[6] = 0;
    {
        double t6 = 0.9e1 * a0 * root2;
        double t8 = 0.6e1 * root * a2;
        double t10 = 0.3e1 * root2 * root;
        double t19 = jmax * root * v0;
        double t34 = 0.3e1 * jmax * root2;
        double t37 = 0.6e1 * jmax * a0 * root;
        t[7] = (-0.2e1 * a2 * a0 - t6 - t8 - t10 - 0.6e1 * jmax2 * x0 + 0.6e1 * jmax2 * xTarget
                + (-0.6e1 * a0 * v0 * jmax - 0.6e1 * t19) * dc + (-t8 - t6 - t10) * da - 0.6e1 * da * dc * t19)
                / (0.6e1 * jmax2 * v0 + (0.3e1 * jmax * a2 + t34 + t37) * dc + (t37 + t34) * dc * da);
    }
}
void Stp7::solveProfileWT(double t[8], double x0, double xTarget, double v0, double a0, double amax, double jmax, double da, double dc) {
    double coeffs[5];
    double a2 = a0 * a0;
    double amax2 = amax * amax;
    {
        double t5 = dc * amax;
        double t10 = a0 * da;
        double t14 = dc * a0;
        double t15 = da * amax;
        double t20 = v0 * jmax;
        double t30 = dc * jmax;
        double t37 = a0 * amax2;
        double t44 = a2 * a0;
        double t64 = jmax * jmax;
        double t82 = v0 * v0;
        double t85 = a2 * a2;
        coeffs[4] = 0.6e1 * da + 0.6e1;
        coeffs[3] = 0.24e2 * a0 + 0.12e2 * t5 + 0.12e2 * dc * da * amax + 0.24e2 * t10;
        coeffs[2] = 0.36e2 * t14 * t15 + 0.36e2 * t5 * a0 + 0.12e2 * t20 * dc
                    + 0.6e1 * amax2 + 0.6e1 * da * amax2 + 0.30e2 * a2 * da
                    + 0.12e2 * t30 * v0 * da + 0.30e2 * a2;
        coeffs[1] = 0.12e2 * t37 + 0.24e2 * t20 * t15 + 0.24e2 * t30 * t10 * v0
                    + 0.12e2 * t44 + 0.24e2 * dc * a2 * t15 + 0.24e2 * t5 * a2
                    + 0.12e2 * t44 * da + 0.24e2 * t20 * t14 + 0.24e2 * t20 * amax
                    + 0.12e2 * t37 * da;
        coeffs[0] = 0.12e2 * dc * amax2 * t20 + 0.24e2 * t5 * t64 * x0
                    - 0.24e2 * t5 * t64 * xTarget + 0.8e1 * t44 * dc * amax
                    + 0.12e2 * a2 * v0 * t30 + 0.6e1 * amax2 * a2
                    + 0.24e2 * t20 * a0 * amax + 0.12e2 * t64 * t82 + 0.3e1 * t85;
    }
    
    Polynomial p(4, coeffs);
    double bestDuration;
    double bestRoot;
    double root;
    double duration;
    bool found = false;
    // iterate through all real roots to find the best valid solution
    for (int i=0; i<p.getDegree(); i++) {
        Complex c = p.getRoot(i);
        if (c.i != 0) continue;
        root = c.r;
        double root2 = root*root;
        t[1] = root / dc / jmax;
        if (t[1] < 0) continue;
        t[3] = (dc * amax + a0 + da * root) / dc / jmax;
        if (t[3] < 0) continue;
        {
            double t4 = pow(root, 0.2e1);
            double t6 = 0.2e1 * root * a0;
            t[6] = (-0.2e1 * amax2 + a2 + t4 + t6 + 0.2e1 * v0 * jmax * dc
                    + (t6 + t4) * da) / jmax / amax / 0.2e1;
            if (t[6] < 0) continue;
        }
        t[7] = amax / jmax;
        duration = t[1]+t[3]+t[6]+t[7];
        if ((!found) || (duration < bestDuration)) {
            bestRoot = root;
            bestDuration = duration;
            found = true;
        }
    }
    if (!found) throw logic_error("No solution found for WT profile!");
    root = bestRoot;
    t[1] = root / dc / jmax;
    t[2] = 0;
    t[3] = (dc * amax + a0 + da * root) / dc / jmax;
    t[4] = 0;
    t[5] = 0;
    {
        double t4 = pow(root, 0.2e1);
        double t6 = 0.2e1 * root * a0;
        t[6] = (-0.2e1 * amax2 + a2 + t4 + t6 + 0.2e1 * v0 * jmax * dc
                + (t6 + t4) * da) / jmax / amax / 0.2e1;
    }
    t[7] = amax / jmax;
}

void Stp7::solveProfileTW(double t[8], double x0, double xTarget, double v0, double a0, double amax, double jmax, double da, double dc) {
    double coeffs[5];
    double a2 = a0 * a0;
    double amax2 = amax * amax;
    {
        double t2 = jmax * da;
        double t17 = jmax * jmax;
        double t18 = amax * t17;
        double t32 = a2 * a2;
        double t36 = v0 * v0;
        coeffs[0] = 0.12e2 * v0 * dc * t2 * a2 + 0.12e2 * v0 * jmax * amax2 * dc * da
                   + 0.8e1 * a2 * a0 * dc * amax + 0.24e2 * t18 * dc * x0
                   - 0.24e2 * t18 * dc * xTarget - 0.24e2 * a0 * v0 * t2 * amax
                   - 0.3e1 * t32 - 0.6e1 * amax2 * a2 - 0.12e2 * t36 * t17;
        coeffs[1] = 0;
        coeffs[2] = 0.12e2 * amax2;
        coeffs[3] = - 0.24e2 * amax * dc;
        coeffs[4] = 0.12e2;
    }
    Polynomial p(4, coeffs);
    double bestDuration;
    double bestRoot;
    double root;
    double duration;
    bool found = false;
    // iterate through all real roots to find the best valid solution
    for (int i=0; i<p.getDegree(); i++) {
        Complex c = p.getRoot(i);
        if (c.i != 0) continue;
        root = c.r;
        double root2 = root*root;
        t[1] = (dc * amax - a0) / da / dc / jmax;
        if (t[1] < 0) continue;
        t[2] = (a2 - amax2 + (amax2 + 0.2e1 * root2) * da
                + (-0.4e1 * root * amax - 0.2e1 * v0 * jmax) * dc * da) / amax / da / jmax / 0.2e1;
        if (t[2] < 0) continue;
        t[3] = root / dc / jmax;
        if (t[3] < 0) continue;
        t[7] = (-dc * amax + root) / dc / jmax;
        if (t[7] < 0) continue;
        duration = t[1]+t[2]+t[3]+t[7];
        if ((!found) || (duration < bestDuration)) {
            bestRoot = root;
            bestDuration = duration;
            found = true;
        }
    }
    if (!found) throw logic_error("No solution found for TW profile!");
    root = bestRoot;
    double root2 = root*root;
    t[1] = (dc * amax - a0) / da / dc / jmax;
    t[2] = (a2 - amax2 + (amax2 + 0.2e1 * root2) * da
            + (-0.4e1 * root * amax - 0.2e1 * v0 * jmax) * dc * da) / amax / da / jmax / 0.2e1;
    t[3] = root / dc / jmax;
    t[4] = 0;
    t[5] = 0;
    t[6] = 0;
    t[7] = (-dc * amax + root) / dc / jmax; 
}



void Stp7::solveProfileTT(double t[8], double x0, double xTarget, double v0, double a0, double amax, double jmax, double da, double dc) {
    double root;
    double a2 = a0 * a0;
    double amax2 = amax * amax;
    {
        double t4 = jmax * v0;
        double t5 = da * dc;
        double t10 = amax2 * da;
        double t14 = amax2 * amax2;
        double t33 = a2 * a2;
        double t35 = jmax * jmax;
        double t36 = t35 * amax;
        double t43 = v0 * v0;
        double coeff[3];
        coeff[2] = 0.24e2;
        coeff[1] = -0.24e2 * a2 + 0.48e2 * t4 * t5 + 0.24e2 * amax2 + 0.48e2 * t10;
        coeff[0] = 0.24e2 * t14 * da + 0.24e2 * t14 + 0.48e2 * jmax * amax2 * dc * v0
                   - 0.24e2 * t4 * da * a0 * amax - 0.12e2 * t4 * t5 * a2
                   + 0.8e1 * a2 * a0 * dc * amax + 0.3e1 * t33
                   - 0.24e2 * t36 * dc * xTarget + 0.24e2 * t36 * dc * x0
                   + 0.12e2 * t35 * t43 + 0.36e2 * amax2 * v0 * jmax * da * dc
                   - 0.18e2 * amax2 * a2 - 0.24e2 * t10 * a2;
        Polynomial p(2,coeff);
        root = p.getSmallestRealRoot();
    }
    
    t[1] = 0.1e1 / da / dc / jmax * (dc * amax - a0);
    t[2] = root / amax / jmax / da;
    t[3] = 0.1e1 / jmax * amax;
    t[4] = 0;
    t[5] = t[3];
    t[6] = (-amax2 * da + 0.2e1 * root + 0.2e1 * jmax * v0 * da * dc - a2 + amax2) / amax / jmax / da / 0.2e1;
    t[7] = t[3];
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
    
    _plannedProfile = true;
    
    
    // test, wether algorithm is correct
    cout << this->getProfileType() << endl;
    cout << "ist: " << _x[7] << " soll: " << xtarget << endl;
    //if (_x[7] != xtarget)
      //  throw logic_error("The planned profile does not reach the goal!");
    
    
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
        oss << _v[0] << ", vmax = " << _vmax << ", a0 = " << _a[0];
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

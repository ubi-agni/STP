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
            if (stillOvershootsTimeInt(tNew, jNew, dir, _x[0], _x[7], _v[0], _a[0]))
                for (int i = 0; i < 8; i++) {_t[i] = tNew[i]; _j[i] = jNew[i]; }
        }
    }

    double da;
    da = _j[1] == _j[3] ? -1 : 1;

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
                if (stillOvershootsTimeInt(tNew, _j, dir, _x[0], _x[7], _v[0], _a[0])) {
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
            solveProfileDD_T(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
            if (_t[2] != 0) _sProfileType = Stp7::PROFILE_TT;
            else _sProfileType = Stp7::PROFILE_WT;
        } else {
            solveProfileDD_W(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
            if (_t[2] != 0) _sProfileType = Stp7::PROFILE_TW;
            else _sProfileType = Stp7::PROFILE_WW;
        }
        // Calculate exact phase duration from given profile t, j
        
        
        // TODO: call solver for double decleration case!
        // t[1], t[2] and j[0..7] already have their correct values!
                
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
    double coeffs[5];
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
        {
            double t22 = 0.3e1 * root2 * jmax;
            double t25 = 0.6e1 * jmax * a0 * root;
            t[3] = (a2 * a0 + 0.3e1 * a2 * da * root - 0.6e1 * jmax2 * x0 - 0.6e1 * root * dc * jmax * v0 + 0.6e1 * jmax2 * xTarget)
                / (0.6e1 * jmax2 * v0 + (0.3e1 * jmax * a2 + t22 + t25) * dc + (t25 + t22) * dc * da);
            if (t[3] < 0) continue;
        }
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
            if (t[7] < 0) continue;
            duration = t[1]+t[3]+t[7];
            if ((!found) || (duration < bestDuration)) {
                bestRoot = root;
                bestDuration = duration;
                found = true;
            }
        }
    }
    if (!found) throw logic_error("No solution found for WW profile!");
    root = bestRoot;
    double root2 = root*root;
    t[1] = root / dc / jmax;
    t[2] = 0;
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
    double coeffs[3];
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
        coeffs[2] = 0.24e2;
        coeffs[1] = -0.24e2 * a2 + 0.48e2 * t4 * t5 + 0.24e2 * amax2 + 0.48e2 * t10;
        coeffs[0] = 0.24e2 * t14 * da + 0.24e2 * t14 + 0.48e2 * jmax * amax2 * dc * v0
                   - 0.24e2 * t4 * da * a0 * amax - 0.12e2 * t4 * t5 * a2
                   + 0.8e1 * a2 * a0 * dc * amax + 0.3e1 * t33
                   - 0.24e2 * t36 * dc * xTarget + 0.24e2 * t36 * dc * x0
                   + 0.12e2 * t35 * t43 + 0.36e2 * amax2 * v0 * jmax * da * dc
                   - 0.18e2 * amax2 * a2 - 0.24e2 * t10 * a2;
    }
    Polynomial p(2,coeffs);
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
        t[1] = 0.1e1 / da / dc / jmax * (dc * amax - a0);
        if (t[1] < 0) continue;
        t[2] = root / amax / jmax / da;
        if (t[2] < 0) continue;
        t[3] = 0.1e1 / jmax * amax;
        if (t[3] < 0) continue;
        t[5] = t[3];
        t[6] = (-amax2 * da + 0.2e1 * root + 0.2e1 * jmax * v0 * da * dc - a2 + amax2) / amax / jmax / da / 0.2e1;
        if (t[6] < 0) continue;
        t[7] = t[3];
        duration = t[1]+t[2]+t[3]+t[5]+t[6]+t[7];
        if ((!found) || (duration < bestDuration)) {
            bestRoot = root;
            bestDuration = duration;
            found = true;
        }
    }
    if (!found) throw logic_error("No solution found for TT profile!");
    root = bestRoot;
    t[1] = 0.1e1 / da / dc / jmax * (dc * amax - a0);
    t[2] = root / amax / jmax / da;
    t[3] = 0.1e1 / jmax * amax;
    t[4] = 0;
    t[5] = t[3];
    t[6] = (-amax2 * da + 0.2e1 * root + 0.2e1 * jmax * v0 * da * dc - a2 + amax2) / amax / jmax / da / 0.2e1;
    t[7] = t[3];
}

/**
 * t[1] and t[2] must be set to the correct values already.
 */
void Stp7::solveProfileDD_T(double t[8], double x0, double xTarget, double v0, double a0, double amax, double jmax, double da, double dc) {
    double coeffs[5];
    {
        double t1 = jmax * jmax;
        double t2 = t1 * jmax;
        double t3 = t[1];
        double t4 = t3 * t3;
        double t5 = t4 * t3;
        double t6 = t2 * t5;
        double t9 = t1 * t1;
        double t10 = t4 * t4;
        double t11 = t9 * t10;
        double t13 = v0 * v0;
        double t16 = t1 * a0;
        double t20 = dc * t1;
        double t24 = t[2];
        double t25 = t24 * a0;
        double t26 = jmax * amax;
        double t29 = amax * amax;
        double t32 = dc * t2;
        double t33 = t4 * t24;
        double t36 = da * dc;
        double t39 = jmax * t3;
        double t43 = t1 * da;
        double t44 = t3 * v0;
        double t47 = t4 * a0;
        double t50 = t36 * t1;
        double t51 = t3 * t24;
        double t55 = a0 * a0;
        double t56 = t55 * a0;
        double t58 = -0.72e2 * t16 * da * t4 + 0.48e2 * t20 * t4 * amax + 0.48e2 * t25 * t26 + 0.24e2 * t29 * a0 + 0.48e2 * t32 * t33 - 0.24e2 * t36 * t6 - 0.24e2 * t36 * t39 * t29 - 0.48e2 * t43 * t44 + 0.72e2 * t47 * t1 - 0.48e2 * t50 * t51 * amax + 0.24e2 * t56;
        double t59 = t1 * t4;
        double t63 = t24 * da;
        double t67 = dc * jmax;
        double t76 = v0 * jmax;
        double t84 = t3 * a0;
        double t87 = t55 * da;
        double t88 = t67 * t3;
        double t91 = t55 * t3;
        double t94 = t55 * t24;
        double t97 = -0.24e2 * t36 * t59 * amax - 0.96e2 * t16 * t63 * t3 + 0.48e2 * t67 * a0 * v0 - 0.96e2 * jmax * a0 * da * t3 * amax + 0.48e2 * t76 * amax + 0.48e2 * dc * t55 * amax + 0.24e2 * t32 * t5 + 0.48e2 * t84 * t26 - 0.72e2 * t87 * t88 + 0.48e2 * t67 * t91 + 0.48e2 * t67 * t94;
        double t103 = t24 * t24;
        double t112 = t9 * t5;
        double t131 = 0.12e2 * t6 * amax + 0.6e1 * t11 + 0.12e2 * t13 * t1 + 0.8e1 * t56 * dc * amax + 0.12e2 * t103 * t9 * t4 + 0.6e1 * t29 * t1 * t4 - 0.6e1 * t11 * da + 0.12e2 * t112 * t24 + 0.30e2 * t4 * t55 * t1 + 0.12e2 * t67 * t29 * v0 - 0.24e2 * t20 * xTarget * amax - 0.12e2 * da * t2 * t5 * amax - 0.6e1 * t29 * da * t59;
        double t141 = amax * a0;
        double t144 = amax * t55;
        double t150 = v0 * t1;
        double t165 = t29 * t3 * a0;
        double t168 = t2 * t4;
        double t175 = a0 * da;
        double t176 = t1 * t3;
        double t180 = t36 * t2;
        double t181 = t33 * a0;
        double t184 = 0.12e2 * t56 * t3 * t67 + 0.12e2 * t56 * t24 * t67 + 0.24e2 * t91 * t1 * t24 + 0.24e2 * t141 * t76 + 0.24e2 * t144 * t39 + 0.24e2 * t144 * t24 * jmax + 0.24e2 * t150 * t84 - 0.30e2 * t87 * t59 - 0.12e2 * t103 * da * t2 * t3 * amax + 0.12e2 * t20 * t103 * a0 * amax + 0.12e2 * t67 * t165 - 0.12e2 * t63 * t168 * amax - 0.24e2 * t87 * t39 * amax - 0.24e2 * t175 * t176 * v0 - 0.36e2 * t180 * t181;
        double t198 = dc * amax;
        double t212 = 0.60e2 * t59 + 0.24e2 * t84 * t67 - 0.120e3 * t175 * t88 + 0.24e2 * t76 * dc + 0.72e2 * t198 * a0 + 0.12e2 * t29 - 0.24e2 * t63 * t176 + 0.60e2 * t55 - 0.72e2 * amax * da * t39 + 0.24e2 * t25 * t67 - 0.12e2 * t43 * t4;
        double t214 = t43 * t3;
        double t217 = t29 * t24;
        double t230 = t47 * amax;
        double t236 = t55 * t55;
        double t250 = 0.24e2 * amax * t2 * t33 - 0.12e2 * t112 * t63 - 0.36e2 * t94 * t214 - 0.12e2 * t217 * t214 - 0.12e2 * t36 * jmax * t165 + 0.12e2 * t67 * t217 * a0 + 0.36e2 * t32 * t181 + 0.6e1 * t29 * t55 - 0.36e2 * t50 * t230 - 0.48e2 * t50 * t51 * t141 + 0.3e1 * t236 - 0.12e2 * t56 * da * t88 - 0.24e2 * t36 * t6 * a0;
        double t258 = t44 * amax;
        double t294 = -0.24e2 * t180 * t51 * v0 - 0.24e2 * t180 * t3 * t103 * a0 + 0.24e2 * t20 * t258 + 0.36e2 * t20 * t230 + 0.12e2 * t32 * t4 * v0 + 0.24e2 * t20 * x0 * amax + 0.24e2 * t32 * a0 * t5 + 0.24e2 * t150 * t25 + 0.12e2 * t55 * v0 * t67 + 0.12e2 * t103 * t55 * t1 + 0.24e2 * t20 * t24 * v0 * amax - 0.12e2 * t36 * t168 * v0 + 0.24e2 * t20 * t24 * t84 * amax - 0.24e2 * t50 * t258;
        
        coeffs[0] = t131 + t184 + t250 + t294;
        coeffs[1] = t58 + t97;
        coeffs[2] = t212;
        coeffs[3] = -0.48e2 * t36 * t39 + 0.24e2 * t198 + 0.48e2 * a0;
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
        
        t[3] = root / dc / jmax;
        if (t[3] < 0) continue;
        t[5] = (dc * amax + a0 - da * dc * jmax * t[1] + root) / dc / jmax;
        if (t[5] < 0) continue;
        {
            double t1 = amax * amax;
            double t3 = a0 * a0;
            double t6 = jmax * jmax;
            double t7 = t[1];
            double t8 = t7 * t7;
            double t9 = t6 * t8;
            double t10 = pow(root, 0.2e1);
            double t14 = t7 * a0 * jmax;
            double t15 = t[2];
            t[6] = (-0.2e1 * t1 + t3 + 0.4e1 * a0 * root + t9 + 0.2e1 * t10 + (0.2e1 * v0 * jmax + 0.2e1 * t14 + 0.2e1 * t15 * a0 * jmax) * dc + (-t9 - 0.2e1 * t6 * t7 * t15) * da + (-0.2e1 * t14 - 0.4e1 * jmax * t7 * root) * da * dc) / amax / jmax / 0.2e1;
            if (t[6] < 0) continue;
        }
        t[7] = amax / jmax;
        if (t[7] < 0) continue;
        duration = t[1]+t[2]+t[3]+t[5]+t[6]+t[7];
        if ((!found) || (duration < bestDuration)) {
            bestRoot = root;
            bestDuration = duration;
            found = true;
        }
    }
    if (!found) throw logic_error("No solution found for ddec ?T profile!");
    root = bestRoot;
    t[3] = root / dc / jmax;
    t[4] = 0;
    t[5] = (dc * amax + a0 - da * dc * jmax * t[1] + root) / dc / jmax;
    {
            double t1 = amax * amax;
            double t3 = a0 * a0;
            double t6 = jmax * jmax;
            double t7 = t[1];
            double t8 = t7 * t7;
            double t9 = t6 * t8;
            double t10 = pow(root, 0.2e1);
            double t14 = t7 * a0 * jmax;
            double t15 = t[2];
            t[6] = (-0.2e1 * t1 + t3 + 0.4e1 * a0 * root + t9 + 0.2e1 * t10 + (0.2e1 * v0 * jmax + 0.2e1 * t14 + 0.2e1 * t15 * a0 * jmax) * dc + (-t9 - 0.2e1 * t6 * t7 * t15) * da + (-0.2e1 * t14 - 0.4e1 * jmax * t7 * root) * da * dc) / amax / jmax / 0.2e1;
    }
    t[7] = amax / jmax;
}

/**
 * t[1] and t[2] must be set to the correct values already.
 */
void Stp7::solveProfileDD_W(double t[8], double x0, double xTarget, double v0, double a0, double amax, double jmax, double da, double dc) {
    double coeffs[5];
    {
        double t1 = jmax * jmax;
        double t2 = t1 * t1;
        double t3 = t2 * jmax;
        double t4 = t3 * dc;
        double t5 = t[2];
        double t6 = t5 * v0;
        double t7 = t[1];
        double t8 = t7 * t7;
        double t9 = t8 * t7;
        double t13 = t1 * jmax;
        double t14 = t13 * dc;
        double t15 = v0 * v0;
        double t16 = a0 * t15;
        double t20 = a0 * t5;
        double t21 = t20 * da;
        double t22 = t2 * t7;
        double t26 = t13 * t7;
        double t27 = t26 * dc;
        double t28 = a0 * a0;
        double t29 = da * t28;
        double t33 = t8 * t5;
        double t41 = t3 * t7 * dc;
        double t43 = t5 * t5;
        double t50 = t8 * t8;
        double t52 = dc * da;
        double t53 = t52 * v0;
        double t56 = xTarget * t1;
        double t58 = a0 * v0;
        double t59 = dc * jmax;
        double t62 = t13 * t9;
        double t67 = x0 * t1;
        double t69 = t13 * t8;
        double t78 = t59 * t7;
        double t81 = t7 * v0;
        double t84 = t43 * a0;
        double t87 = t28 * a0;
        double t89 = da * t1;
        double t92 = a0 * da;
        double t93 = t1 * t8;
        double t96 = t5 * t7;
        double t102 = t8 * a0;
        double t105 = t7 * t28;
        double t108 = t89 * t7;
        double t111 = t5 * t28;
        double t114 = 0.144e3 * t56 + 0.144e3 * t58 * t59 + 0.72e2 * t62 * t52 - 0.144e3 * t6 * t1
                - 0.144e3 * t67 + 0.72e2 * t69 * t52 * t5 + 0.72e2 * t26 * t52 * t43
                + 0.144e3 * t14 * t33 + 0.144e3 * t29 * t78 - 0.144e3 * t81 * t1
                - 0.72e2 * t84 * t1 - 0.48e2 * t87 - 0.144e3 * t89 * t81
                - 0.216e3 * t92 * t93 - 0.144e3 * t96 * a0 * t1 + 0.72e2 * t14 * t9
                - 0.216e3 * t102 * t1 + 0.144e3 * t105 * t59 - 0.288e3 * t20 * t108
                + 0.144e3 * t111 * t59;
        double t116 = da * t2;
        double t120 = t7 * t15;
        double t124 = 0.72e2 * t4 * t6 * t9 - 0.72e2 * t14 * t16 * t7 - 0.288e3 * t21 * t22 * x0
                + 0.144e3 * t27 * t29 * x0 + 0.144e3 * t4 * t33 * x0 - 0.144e3 * t27 * t29 * xTarget
                + 0.72e2 * t41 * da * x0 * t43 + 0.288e3 * t21 * t22 * xTarget + 0.36e2 * t3 * t50 * t53
                + 0.36e2 * t111 * t116 * t9 + 0.72e2 * t116 * t120 * t5;
        double t125 = t2 * t1;
        double t129 = t43 * t5;
        double t133 = t43 * t43;
        double t143 = t28 * t28;
        double t154 = t69 * dc;
        double t155 = da * t87;
        double t159 = t3 * t9;
        double t164 = t3 * t8 * dc;
        double t165 = da * t5;
        double t169 = -0.18e2 * t125 * t50 * t43 - 0.36e2 * t129 * t125 * t9 - 0.18e2 * t133 * t125 * t8
                - 0.72e2 * t14 * t15 * v0 + 0.36e2 * t28 * t15 * t1 + 0.12e2 * t43 * t143 * t1
                - 0.48e2 * t67 * t87 + 0.48e2 * t56 * t87 + 0.27e2 * t8 * t143 * t1
                - 0.84e2 * t154 * t155 * t5 + 0.72e2 * t159 * t52 * x0 - 0.72e2 * t164 * t165 * xTarget;
        double t183 = t143 * a0;
        double t190 = t2 * t8;
        double t194 = t9 * v0;
        double t198 = t8 * t15;
        double t210 = -0.144e3 * t116 * t81 * x0 - 0.72e2 * t159 * t52 * xTarget
                + 0.144e3 * t116 * t81 * xTarget - 0.144e3 * t4 * t33 * xTarget
                - 0.6e1 * t183 * da * t78 - 0.72e2 * t14 * t16 * t5 - 0.72e2 * t21 * t190 * v0
                - 0.72e2 * t116 * t194 * a0 - 0.36e2 * t198 * t2 + 0.18e2 * t50 * t28 * t2
                - 0.72e2 * t43 * t15 * t2 - 0.18e2 * t133 * t28 * t2;
        double t214 = t8 * v0;
        double t218 = xTarget * t2;
        double t219 = t96 * a0;
        double t222 = t87 * t5;
        double t229 = x0 * t2;
        double t235 = t2 * t43;
        double t252 = t2 * t5;
        double t255 = 0.72e2 * t164 * t165 * x0 - 0.36e2 * t14 * t214 * t28
                + 0.144e3 * t218 * t219 - 0.60e2 * t14 * t222 * t8
                + 0.144e3 * t14 * t111 * x0 - 0.144e3 * t229 * t219
                + 0.144e3 * t14 * t105 * x0 - 0.216e3 * t81 * t235 * a0
                + 0.36e2 * t4 * t50 * v0 - 0.144e3 * t14 * t105 * xTarget
                + 0.72e2 * t4 * x0 * t9 - 0.48e2 * t14 * t9 * t87 + 0.36e2 * t9 * t28 * t252;
        double t271 = t2 * a0;
        double t279 = t50 * t2;
        double t287 = da * t43;
        double t291 = -0.6e1 * t7 * t183 * t59 + 0.144e3 * t218 * t81 + 0.216e3 * t218 * t102
                - 0.6e1 * t5 * t183 * t59 - 0.6e1 * t143 * v0 * t59 - 0.72e2 * t194 * t271
                - 0.144e3 * t120 * t252 - 0.36e2 * t8 * t28 * t235 + 0.18e2 * t29 * t279
                - 0.72e2 * t229 * t84 - 0.48e2 * t62 * t52 * t87 - 0.48e2 * t27 * t287 * t87;
        double t296 = da * v0;
        double t311 = t143 * t5;
        double t316 = t1 * t87;
        double t324 = -0.144e3 * t229 * t6 + 0.72e2 * t218 * t84 - 0.36e2 * t154 * t296 * t28
                - 0.72e2 * t4 * xTarget * t9 - 0.72e2 * t129 * v0 * t271 - 0.36e2 * t116 * t198
                - 0.72e2 * t129 * t28 * t22 + 0.30e2 * t311 * t108 - 0.216e3 * t229 * t102
                + 0.24e2 * t6 * t316 + 0.27e2 * t143 * da * t93 + 0.144e3 * t218 * t6;
        double t328 = t7 * t1;
        double t333 = t159 * dc;
        double t334 = t165 * v0;
        double t342 = t287 * a0;
        double t350 = t1 * t28;
        double t371 = t28 * v0;
        double t374 = 0.216e3 * t154 * t21 + 0.144e3 * t27 * t342 + 0.144e3 * t27 * t334
                - 0.144e3 * t89 * t81 * a0 - 0.216e3 * t350 * t165 * t7 + 0.216e3 * t14 * t20 * t8
                - 0.72e2 * t1 * t15 - 0.144e3 * t1 * t5 * t58 - 0.144e3 * t328 * t58
                - 0.144e3 * t350 * t96 + 0.72e2 * t59 * t7 * t87 + 0.72e2 * t59 * t222
                + 0.72e2 * t59 * t371;
        double t375 = t9 * t2;
        double t407 = -0.72e2 * t375 * t165 + 0.72e2 * t14 * t214 + 0.144e3 * t14 * t9 * a0
                - 0.180e3 * t350 * da * t8 - 0.36e2 * t279 + 0.144e3 * t62 * t52 * a0
                + 0.72e2 * t59 * t155 * t7 - 0.18e2 * t143 + 0.72e2 * t69 * t53
                - 0.72e2 * t375 * t5 - 0.72e2 * t190 * t43 - 0.180e3 * t93 * t28
                - 0.72e2 * t1 * t43 * t28 - 0.36e2 * t279 * da;
        double t413 = x0 * x0;
        double t428 = -0.144e3 * t229 * t81 + 0.24e2 * t311 * t328 + 0.24e2 * t81 * t316
                + 0.72e2 * t333 * t334 - 0.144e3 * t14 * t111 * xTarget + 
                + 0.24e2 * t89 * t81 * t87 - 0.72e2 * t413 * t2 + 0.72e2 * t333 * t342
                - 0.72e2 * t41 * da * xTarget * t43 + 0.144e3 * t14 * t58 * x0 - 0.72e2 * t27 * t165 * t371;
        double t435 = da * t129;
        double t442 = xTarget * xTarget;
        double t482 = 0.144e3 * t164 * t296 * t43 - 0.144e3 * t14 * t58 * xTarget + 0.72e2 * t41 * t435 * v0
                - 0.216e3 * t92 * t190 * x0 - 0.72e2 * t442 * t2 +  
                + 0.108e3 * t164 * t435 * a0 - 0.72e2 * t27 * t92 * t15 + 0.216e3 * t92 * t190 * xTarget
                + 0.36e2 * t41 * da * t133 * a0 + 0.144e3 * t229 * xTarget - 0.144e3 * t214 * t252 * a0 + t143 * t28;
        coeffs[0] = t124 + t169 + t210 + t255 + t291 + t324 + t428 + t482;
        coeffs[1] = 0.;
        coeffs[2] = t374 + t407;
        coeffs[3] = t114;
        coeffs[4] = (0.72e2 * t20 * t59
                + 0.72e2 * t92 * t78 + 0.72e2 * v0 * dc * jmax - 0.36e2 * t93
                - 0.72e2 * t165 * t328 - 0.36e2 * t89 * t8 + 0.72e2 * t7 * a0 * t59 - 0.36e2 * t28);
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
        {
            double t1 = pow(root, 0.2e1);
            double t4 = jmax * jmax;
            double t9 = t[1];
            double t13 = t[2];
            double t17 = t13 * t13;
            double t23 = a0 * a0;
            double t26 = a0 * t4;
            double t29 = jmax * t1;
            double t35 = t4 * jmax;
            double t36 = t9 * t9;
            double t37 = t35 * t36;
            double t58 = 0.3e1 * t37;
            t[3] = (0.6e1 * t1 * root + 0.6e1 * x0 * t4 - 0.6e1 * xTarget * t4 + 0.6e1 * t9 * v0 * t4
                    + 0.6e1 * t13 * v0 * t4 + 0.3e1 * t17 * a0 * t4 + 0.6e1 * t1 * a0 - t23 * a0
                    + 0.6e1 * t13 * t9 * t26 + (-0.6e1 * t29 * t9 + 0.3e1 * t9 * t23 * jmax
                    - 0.3e1 * t37 * t13 - 0.3e1 * t17 * t35 * t9) * da * dc)
                    / (-0.6e1 * t4 * v0 - 0.6e1 * t4 * t9 * a0 - 0.6e1 * t4 * t13 * a0
                    + (-0.6e1 * t29 + 0.3e1 * jmax * t23 + t58) * dc - 0.6e1 * t26 * da * t9
                    + (t58 + 0.6e1 * t35 * t13 * t9) * dc * da);
            if (t[3] < 0) continue;
        }
        {
            double t1 = jmax * jmax;
            double t2 = t1 * root;
            double t3 = t[1];
            double t4 = t3 * t3;
            double t6 = 0.3e1 * t2 * t4;
            double t7 = a0 * a0;
            double t18 = 0.9e1 * t4 * a0 * t1;
            double t19 = t[2];
            double t25 = 0.6e1 * t3 * v0 * t1;
            double t26 = t19 * t19;
            double t30 = t19 * t3;
            double t31 = t1 * a0;
            double t32 = t30 * t31;
            double t36 = 0.6e1 * t3 * t7 * jmax;
            double t43 = jmax * root;
            double t46 = jmax * t1;
            double t49 = 0.3e1 * t46 * t4 * t3;
            double t50 = t46 * t4;
            double t51 = t50 * t19;
            double t55 = 0.6e1 * t43 * a0 * t3;
            double t73 = t6 + 0.2e1 * t7 * a0 + 0.3e1 * root * t7 - 0.6e1 * xTarget * t1
                    + 0.6e1 * x0 * t1 + t18 + 0.6e1 * t19 * v0 * t1 + t25 + 0.3e1 * t26 * a0 * t1
                    + 0.6e1 * t32 + (-t36 - 0.6e1 * t19 * t7 * jmax - 0.6e1 * a0 * v0 * jmax
                    - 0.6e1 * t43 * v0 - t49 - 0.6e1 * t51 - t55 - 0.6e1 * t43 * t19 * a0) * dc
                    + (t25 + t18 + 0.12e2 * t32 + 0.6e1 * t2 * t30 + t6) * da 
                    + (-t55 - t36 - t49 - 0.3e1 * t51 - 0.3e1 * t26 * t46 * t3) * dc * da;
            double t82 = pow(root, 0.2e1);
            double t87 = 0.3e1 * t50;
            t[5] = t73 / (-0.6e1 * t1 * v0 - 0.6e1 * t1 * t3 * a0 - 0.6e1 * t1 * t19 * a0
                    + (-0.6e1 * jmax * t82 + 0.3e1 * jmax * t7 + t87) * dc
                    - 0.6e1 * t31 * da * t3 + (t87 + 0.6e1 * t46 * t19 * t3) * dc * da);
        }
        if (t[5] < 0) continue;
        t[7] = root / dc / jmax;
        if (t[7] < 0) continue;
        duration = t[1] +t[2] + t[3] + t[5] + t[7];
        if ((!found) || (duration < bestDuration)) {
            bestRoot = root;
            bestDuration = duration;
            found = true;
        }
    }
    if (!found) throw logic_error("No solution found for ddec ?W profile!");
    root = bestRoot;
    t[4] = 0;
    t[6] = 0;
    {
            double t1 = pow(root, 0.2e1);
            double t4 = jmax * jmax;
            double t9 = t[1];
            double t13 = t[2];
            double t17 = t13 * t13;
            double t23 = a0 * a0;
            double t26 = a0 * t4;
            double t29 = jmax * t1;
            double t35 = t4 * jmax;
            double t36 = t9 * t9;
            double t37 = t35 * t36;
            double t58 = 0.3e1 * t37;
            t[3] = (0.6e1 * t1 * root + 0.6e1 * x0 * t4 - 0.6e1 * xTarget * t4 + 0.6e1 * t9 * v0 * t4
                    + 0.6e1 * t13 * v0 * t4 + 0.3e1 * t17 * a0 * t4 + 0.6e1 * t1 * a0 - t23 * a0
                    + 0.6e1 * t13 * t9 * t26 + (-0.6e1 * t29 * t9 + 0.3e1 * t9 * t23 * jmax
                    - 0.3e1 * t37 * t13 - 0.3e1 * t17 * t35 * t9) * da * dc)
                    / (-0.6e1 * t4 * v0 - 0.6e1 * t4 * t9 * a0 - 0.6e1 * t4 * t13 * a0
                    + (-0.6e1 * t29 + 0.3e1 * jmax * t23 + t58) * dc - 0.6e1 * t26 * da * t9
                    + (t58 + 0.6e1 * t35 * t13 * t9) * dc * da);
        }
        {
            double t1 = jmax * jmax;
            double t2 = t1 * root;
            double t3 = t[1];
            double t4 = t3 * t3;
            double t6 = 0.3e1 * t2 * t4;
            double t7 = a0 * a0;
            double t18 = 0.9e1 * t4 * a0 * t1;
            double t19 = t[2];
            double t25 = 0.6e1 * t3 * v0 * t1;
            double t26 = t19 * t19;
            double t30 = t19 * t3;
            double t31 = t1 * a0;
            double t32 = t30 * t31;
            double t36 = 0.6e1 * t3 * t7 * jmax;
            double t43 = jmax * root;
            double t46 = jmax * t1;
            double t49 = 0.3e1 * t46 * t4 * t3;
            double t50 = t46 * t4;
            double t51 = t50 * t19;
            double t55 = 0.6e1 * t43 * a0 * t3;
            double t73 = t6 + 0.2e1 * t7 * a0 + 0.3e1 * root * t7 - 0.6e1 * xTarget * t1
                    + 0.6e1 * x0 * t1 + t18 + 0.6e1 * t19 * v0 * t1 + t25 + 0.3e1 * t26 * a0 * t1
                    + 0.6e1 * t32 + (-t36 - 0.6e1 * t19 * t7 * jmax - 0.6e1 * a0 * v0 * jmax
                    - 0.6e1 * t43 * v0 - t49 - 0.6e1 * t51 - t55 - 0.6e1 * t43 * t19 * a0) * dc
                    + (t25 + t18 + 0.12e2 * t32 + 0.6e1 * t2 * t30 + t6) * da 
                    + (-t55 - t36 - t49 - 0.3e1 * t51 - 0.3e1 * t26 * t46 * t3) * dc * da;
            double t82 = pow(root, 0.2e1);
            double t87 = 0.3e1 * t50;
            t[5] = t73 / (-0.6e1 * t1 * v0 - 0.6e1 * t1 * t3 * a0 - 0.6e1 * t1 * t19 * a0
                    + (-0.6e1 * jmax * t82 + 0.3e1 * jmax * t7 + t87) * dc
                    - 0.6e1 * t31 * da * t3 + (t87 + 0.6e1 * t46 * t19 * t3) * dc * da);
        }
        t[7] = root / dc / jmax;
}

/**
 * This function stretches the computed profile to a new (longer) duration.
 */
double Stp7::scaleToDuration(double newDuration) {
    convertTimePointsToIntervalls();
    
    splitNoCruiseProfileTimeInt(_t, _j, _a[0]);

    double a_dummy, v_dummy, x_dummy;
    
    double v3;
    calcjTracksTimeInt(_t, _j, 3, _x[0], _v[0], _a[0], x_dummy, v3, a_dummy);
    double dir = sign(v3); // TODO: what happens when dir=0?
    
    // check whether we must use the double deceleration branch
    bool useDDec = false;
    double t_orig[8]; for (int i = 0; i < 8; i++) t_orig[i] = _t[i];
    double j_orig[8]; for (int i = 0; i < 8; i++) j_orig[i] = _j[i];
    {
       // Tests for a given profile, whether stretching it to the time T would
       // nead a double deceleration profile.    
       // TODO TODO TODO:
       // folgenden einfachen algorithmus verwenden:
       // Profil erzeugen: erst a auf null, dann v auf null
       // ist dieses profil zu langsam --> double deceleration / normal

        if (sign(_j[3]) != sign(_j[5])) {
            // that was easy - it already is a double dec. profile :)
            useDDec = true;
        } else {
            // If the velocity change within the deceleration part is larger in magnitude than
            // the velocity change within acceleration part (starting from max velocity,
            // i.e. at a=0, the cutting process in findProfileNormal may lead to the
            // situation, where no more area can be cut from the acceleration part,
            // although the desired duration T is not yet reached. In this case we have
            // to switch to the double deceleration branch as well.
            // We check whether we are still too early at the target, when employing a
            // full stop profile, reducing acceleration to zero immidiately in the first
            // phase. 
            double j1,t1;    
            if (_a[0] == 0) {
                j1 = _j[5]; // direction of double deceleration
                t1 = 0;
            } else {
                // jerk to decrease acceleration to zero
                j1 = -sign(_a[0]) * _jmax; 
                // time needed to reach zero acceleration
                t1 = fabs(_a[0]/j1); 
            }

            // position and velocity reached after this time
            double a1, v1, x1;
            calcjTrack(t1, _x[0], _v[0], _a[0], j1, x1, v1, a1); // a1 == 0
            // profile to reach zero velocity, starting from v1
            Stp3 stp3Dec;
            stp3Dec.planFastestProfile(v1, 0, a1, _amax, _jmax);
            double tdec[4], jdec[4];
            stp3Dec.getTimeIntArray(tdec);
            stp3Dec.getAccArray(jdec);
            
            // If the a(t) profile in deceleration part has the same direction as before,
            // we may need to switch to a double deceleration profile. 
            if (sign(jdec[1]) == sign(_j[5])) { // we may need to switch
                if (j1 == jdec[3]) { 
                    _t[1] = 0; _t[2] = 0; _t[3] = t1; _t[4] = 0;
                    _t[5] = tdec[1]; _t[6] = tdec[2]; _t[7] = tdec[3];
                    _j[1] = jdec[1]; _j[2] = jdec[2]; _j[3] = jdec[3];
                    _j[4] = 0; _j[5] = jdec[1]; _j[6] = jdec[2]; _j[7] = jdec[3];
                } else {
                    _t[1] = t1; _t[2] = 0; _t[3] = 0; _t[4] = 0;
                    _t[5] = tdec[1]; _t[6] = tdec[2]; _t[7] = tdec[3];
                    _j[1] = jdec[1]; _j[2] = jdec[2]; _j[3] = jdec[3];
                    _j[4] = 0; _j[5] = jdec[1]; _j[6] = jdec[2]; _j[7] = jdec[3];
                }
                // insert cruising phase, such that the target is still reached
                adaptProfile (_t, _j, _x[7], _x[0], _v[0], _a[0]);
                useDDec = stillTooShort (_t, newDuration);
            } else {
                useDDec = false;
            }
        }
    }
    if (!useDDec) {
        for (int i = 0; i < 8; i++) {
            _t[i] = t_orig[i];
            _j[i] = j_orig[i];
        }
    }
    
    if (useDDec) {
        // double deceleration branch
        findProfileTypeStretchDoubleDec(newDuration);
        // extend simple profile computed in useDoubleDeceleration() to
        // wedge-shaped profile
        if (_t[1] != 0 && _t[3] == 0) _t[3] = 1;
        if (_t[1] == 0 && _t[3] != 0) _t[1] = 1;
        _sProfileType = getProfileString(_t);
        if (_sProfileType == Stp7::PROFILE_TT || _sProfileType == Stp7::PROFILE_WT) {
            TS_WARN("stretch ddec ?T - TODO");
            //solveProfileTT(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
        } else if (_sProfileType == Stp7::PROFILE_TW || _sProfileType == Stp7::PROFILE_WW) {
            TS_WARN("stretch ddec ?W - TODO");
            //solveProfileTW(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
        } else throw invalid_argument("Unknown profile!");
        _bIsddec = true;
        _bHasCruise = (_t[4] != 0);
    } else {
        // normal profile branch
        findProfileTypeStretchCanonical(newDuration);
        _sProfileType = getProfileString(_t);
        if (_sProfileType == Stp7::PROFILE_TT) {
            TS_WARN("stretch canonical TT - TODO");
            //solveProfileTT(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
        } else if (_sProfileType == Stp7::PROFILE_TW) {
            TS_WARN("stretch canonical TW - TODO");
            //solveProfileTW(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
        } else if (_sProfileType == Stp7::PROFILE_WT) {
            TS_WARN("stretch canonical WT - TODO");
            //solveProfileWT(_t, _x[0], _x[7], _v[0], _a[0], _amax, _jmax, da, dir);
        } else if (_sProfileType == Stp7::PROFILE_WW) {
            TS_WARN("stretch canonical WW - TODO");
            //solveProfileWW(_t, _x[0], _x[7], _v[0], _a[0], _jmax, da, dir);
        } else throw invalid_argument("Unknown profile!");
        _bIsddec = false;
        this->_bHasCruise = (_t[4] != 0);
    }
    convertTimeIntervallsToPoints();
    
    return getDuration();
}

void Stp7::adaptProfile(double t[8], double j[8], double xtarget, double x0, double v0, double a0) {
    // Given a profile (t,j) where acceleration and deceleration phase was
    // already changed (cutted or shifted) such that velocity v(3) is smaller
    // in magnitude than before, this function extends the cruising phase (or
    // inserts one), such that the target is reach again.
    // This is a simple linear equation...
    double a_dummy, v_dummy, x_dummy;
    double xend, v3new;
    calcjTracksTimeInt(t, j, 7, x0, v0, a0, xend, v_dummy, a_dummy);
    calcjTracksTimeInt(t, j, 3, x0, v0, a0, x_dummy, v3new, a_dummy);
    //  enlarge cruising time, such that area below velocity profile equals dp again
    t[4] = t[4] + (xtarget - xend) / v3new;
}

bool Stp7::stillTooShort(double t[8], double newDuration) {
     return (t[1]+t[2]+t[3]+t[4]+t[5]+t[6]+t[7] < newDuration);
}

void Stp7::findProfileTypeStretchCanonical(double newDuration) {
    // find correct profile by cutting pieces and descending to shorter profiles

    _sProfileType = getProfileString(_t);
    
    // if profile type does not change, we just insert cruising phase into t
    double t_orig[8];
    for (int i = 0; i < 8; i++) t_orig[i] = _t[i];
    if (_t[4] == 0) t_orig[4]=1;
    
    if (_sProfileType == PROFILE_TT) {
        // cut out smaller a=const. part
        double dt = min(_t[2], _t[6]);
        _t[2] = _t[2] - dt;
        _t[6] = _t[6] - dt;
        adaptProfile (_t, _j, _x[7], _x[0], _v[0], _a[0]);
        if (stillTooShort(_t, newDuration)) {
            // recursively calling this function even cuts further
            findProfileTypeStretchCanonical(newDuration);
        } else {
            // now we stop after duration time newDuration, hence profile stays TT
           for (int i = 0; i < 8; i++) _t[i] = t_orig[i]; // allow for a cruising phase
        }
        return;
    }

    if (_sProfileType == PROFILE_WW) {
        for (int i = 0; i < 8; i++) _t[i] = t_orig[i]; // allow for a cruising phase
        return; // nothing to do, WW stays WW anytime
    }

    if (_sProfileType == PROFILE_WT) {
        double a1 = _a[0] + _j[1]*_t[1];
        double dt_w = min(_t[1],_t[3]);
        double area_w_max = 0.5*dt_w*dt_w*_jmax;
        if (_t[1] > _t[3]) area_w_max += fabs(2.*_a[0]*dt_w);
        double area_t_max = _t[6]*_amax;
        if (area_w_max > area_t_max) {
            // we will cut out the whole t(6) WT -> WW
            _t[6] = 0;
            double dt = (fabs(a1)-sqrt(a1*a1-area_t_max))/_jmax;
            _t[1] = _t[1]-dt;
            _t[3] = _t[3]-dt;
            adaptProfile(_t,_j,_x[7],_x[0],_v[0],_a[0]);
            if (stillTooShort(_t,newDuration)) {
                _sProfileType = PROFILE_WW; // type switches to WW
            } else {
                // now we stop after duration time newDuration, hence profile stays WT
                for (int i = 0; i < 8; i++) _t[i] = t_orig[i]; // allow for a cruising phase
            }
        } else {
            // nothing to cut out, stays at WT
            for (int i = 0; i < 8; i++) _t[i] = t_orig[i]; // allow for a cruising phase
        }
        return;
    }
    
    if (_sProfileType == PROFILE_TW) {
        double a5 = _j[5]*_t[5];
        double area_w_max = fabs(_t[5]*a5);
        double area_t_max = _t[2]*_amax;
        if (area_w_max > area_t_max) {
            // we will cut out the whole t(2)
            _t[2] = 0;
            _t[5] = sqrt((area_w_max-area_t_max)/fabs(_j[5]));
            _t[7] = _t[5];
            adaptProfile(_t,_j,_x[7],_x[0],_v[0],_a[0]);
            if (stillTooShort(_t,newDuration)) {
                _sProfileType = PROFILE_WW; // type switches to WW
            } else {
                // now we stop after duration time newDuration, hence profile stays TW
                for (int i = 0; i < 8; i++) _t[i] = t_orig[i]; // allow for a cruising phase
            }
        } else {
            // nothing to cut out, stays at WT
            for (int i = 0; i < 8; i++) _t[i] = t_orig[i]; // allow for a cruising phase
        }
        return;
    }
    
    return;
}

void Stp7::findProfileTypeStretchDoubleDec(double newDuration) {
    // find correct double deceleration profile by shifting area from second
    // deceleration to first deceleration part.
    // We can get two type of deceleration profiles here:
    // 1) The time-optimal profile was already double deceleraton, leading to a(3) ~= 0
    // 2) all other profiles: a(3) == 0, there might be profile [0 0 t3] / [t1 0 0]
    
    double v_dummy, x_dummy;
    double a3, a7;
    calcjTracksTimeInt(_t, _j, 3, _x[0], _v[0], _a[0], x_dummy, v_dummy, a3);
    
    double tdec[4];
    
    if (!isZero(a3)) {
        // 1) Time-optiomal profile was double deceleration already
        // In this case we must check, whether we can reach a3 = 0 (to insert
        // a cruising phase
        double tz = fabs(a3/_jmax); // time needed to reach zero acceleration
        // try to shorten deceleration profile, such that a3 reaches zero in between
        tdec[1] = _t[5]+tz; tdec[2] = _t[6]; tdec[3] = _t[7];
        removeAreaTimeInt(tdec, fabs(a3*tz), _amax, _jmax);
        // adapt profile to reach target again
        double tn[8];
        tn[1] = _t[1]; tn[2] = _t[2]; tn[3] = _t[3]; tn[4] = 0;
        tn[5] = tdec[1]; tn[6] = tdec[2]; tn[7] = tdec[3];
        adaptProfile (tn, _j, _x[7], _x[0], _v[0], _a[0]);
        // if target is overshooted if trying to set a3=0, 
        // adaptProfile will return negative time tn(4) (and we must keep the current profile)
        if (tn[4] < 0 || !stillTooShort(tn,newDuration))
            return; // keep existing profile
        for (int i = 0; i < 6; i++) _t[i] = tn[i]; // use adapted profile for further checks
    }

    // Compute current velocity decrease achieved during first and second part
    double curFirst, curLast, t_0acc;
    calcjTracksTimeInt(_t, _j, 3, 0, 0, _a[0], x_dummy, curFirst, a3); // a3=0
    calcjTracksTimeInt(&(_t[4]), _j, 3, 0., 0., 0., x_dummy, curLast, a7); // a7=0
    //////////////// WARUM nicht _j[4] ???
    curFirst = fabs(curFirst); curLast = fabs(curLast);

    // We enlarge curFirst such, that it contains the area of the (full) triangle 
    // which reaches peak acceleration a2
    if (_a[0] !=0 && sign(_a[0]) == sign(_j[7])) {
        // time needed to reach zero acceleration
        t_0acc = fabs(_a[0]/_j[1]);
        // add area on the other side of x-axis 
        // (because it was substracted during integration)
        curFirst = curFirst + fabs(_a[0])*t_0acc/2;
    } else {
        // time needed to reach zero acceleration (now backwards)
        t_0acc = -fabs(_a[0]/_j[1]);
        // add (virtual) area of the missing triangle
        curFirst = curFirst + fabs(_a[0]*t_0acc/2);
    }
    
    double wedgeMax = _amax*_amax/_jmax;
    double deltaFirst, deltaLast, deltaV, tacc[4], tn[8];
    
    while (1) {
        // area needed to extend first part to full wedge
        deltaFirst = wedgeMax - curFirst;
        if (_t[2] == 0 && !isZero(deltaFirst)) { // first part is not yet full wedge 
            if (_t[6] == 0) {
                deltaLast = curLast; // area available in second part
                // if last part has not enough area to extend first one to full
                // triangle, the profile will keep WW shape
                if (deltaFirst >= deltaLast) return;
            } else {
                deltaLast = _t[6]*_amax; // area below const-trapezoidal part
            }
            deltaV = min(deltaFirst, deltaLast);
        } else {
            if (_t[2] == 0) _t[2] = 1; // allow const-part in trapez
            if (_t[6] == 0) return; // profile will keep TW shape
            deltaV = _t[6]*_amax; // area below const-trapezoidal part
        }
        
        addAreaTimeInt (t_0acc, curFirst + deltaV - wedgeMax, _amax,_jmax, tacc);
        tdec[1] = _t[5]; tdec[2] = _t[6]; tdec[3] = _t[7];
        removeAreaTimeInt (tdec, deltaV, _amax,_jmax);
        double tn[8];
        tn[1] = tacc[1]; tn[2] = tacc[2]; tn[3] = tacc[3]; tn[4] = _t[4];
        tn[5] = tdec[1]; tn[6] = tdec[2]; tn[7] = tdec[3];
        adaptProfile (tn, _j, _x[7], _x[0], _v[0], _a[0]);
        // if we overshoot in time, t contains the correct profile
        if (~stillTooShort(tn,newDuration)) return;
        // otherwise continue probing with adapted profile
        for (int i = 0; i < 6; i++) _t[i] = tn[i]; // use adapted profile for further checkst = tn;
        curFirst = curFirst + deltaV;
        curLast  = curLast  - deltaV;
    }
}

void Stp7::addAreaTimeInt(double deltaT1, double deltaV, double amax, double jmax, double t[4]) {
    // Compute a profile [t1 t2 t3] such that its area is wedgeMax + deltaV.
    // deltaT1 is extra time added to t1
    // The result is written in t[1..3].
    double tmax = amax/jmax;
    if (deltaV >= 0) { // full wedge + const trapezoidal part
        t[1] = tmax + deltaT1;
        t[2] = deltaV/amax;
        t[3] = tmax;
    } else {
        double deltaT = tmax - sqrt (tmax*tmax + deltaV/jmax);
        t[1] = tmax + deltaT1 - deltaT;
        t[2] = 0;
        t[3] = tmax - deltaT;
    }
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
        if (stillOvershootsTimeInt(t, j, dir, x0, xTarget, v0, a0)) {
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
            if (stillOvershootsTimeInt(t, j, dir, x0, xTarget, v0, a0)) {
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
            if (stillOvershootsTimeInt(t, j, dir, x0, xTarget, v0, a0)) {
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

void Stp7::splitNoCruiseProfileTimeInt(double t[8], double j[8], double a0) {
    // In case of a profile without cruising phase, the time intervalls
    // t(3) and t(5) might be joined together into one of them so the other
    // one is zero. This can only occour if j(3) and j(5) have the same
    // sign.
    // For the stretching algorithm, we need to split the time intervall up
    // so the acc-graph reaches zero after t(3).
    if (t[4] != 0) return;
    if (j[3] != j[5]) return;
    double tsum = t[3] + t[5];
    t[3] = fabs((a0 + j[1]*t[1]) / j[3]);   // = -a2/j(3)
    t[5] = tsum - t[3];
}
    
bool Stp7::stillOvershootsTimeInt(double t[8], double j[8], int dir,
                            double x0, double xTarget, double v0, double a0) {
    double v_dummy, a_dummy, xEnd;
    Stp7::calcjTracksTimeInt(t, j, 7, x0, v0, a0, xEnd, v_dummy, a_dummy);
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
    //cout << this->getProfileType() << endl;
    //cout << "delta p: " << _x[7]-xtarget << endl;
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

void Stp7::getJerkArray(double j[8]) const {
    if (!_plannedProfile)
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    for (int i = 0; i < 8; i++) j[i] = _j[i];
}

void Stp7::getTimeArray(double t[8]) const {
    if (!_plannedProfile)
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    for (int i = 0; i < 8; i++) t[i] = _t[i];
}

void Stp7::getTimeIntArray(double t[8]) const {
    if (!_plannedProfile)
        throw invalid_argument("Consider to call planFastestProfile(.) first.");
    t[0] = 0; t[1] = _t[1];
    for (int i = 2; i < 8; i++) t[i] = _t[i] - _t[i-1];
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

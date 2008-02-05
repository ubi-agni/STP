/**
 * \file stp7Formulars.cc
 * \author Erik Weitnauer
 */

#include "stp7Formulars.h"
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

using namespace std;

/**
 * .
 * @param[out] t calculated time intervalls
 * @param[in] type a value amoung Stp7::PROFILE_{TT, WT, TW, WW}
 * @param[in] bCruise true when profile has cruising phase
 * @param[in] bDoubleDec true when profile is double decceleration profile
 * @param[in] bDecAcc must acceleration be reduced first since a0 > amax?
 * @param[in] bMoveForward false when an overshooting of target is inevitable 
 * @param[in] x0 start position
 * @param[in] xTarget target position
 * @param[in] v0 initial velocity
 * @param[in] vmax limit for velocity
 * @param[in] a0 initial acceleration
 * @param[in] amax limit for acceleration
 * @param[in] jmax limit for jerk
 * @param[in] stretchToTime If the profile should be stretched, this will be the
 * resulting duration of movement. For time optimal profiles this value must be 
 * set to zero.
 */
void Stp7Formulars::solveProfile(double t[8],
        string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double stretchToTime) throw(logic_error) {
    // get the coeffients of the polynomial we need to find the roots of
    double coeffs[7] = {0.,0.,0.,0.,0.,0.};
    calcCoeffs(coeffs, type, bCruise, bDoubleDec, bDecAcc, bMoveForward, x0, xTarget,
            v0, vmax, a0, amax, jmax, stretchToTime, t);
    // create polynomial
    Polynomial p(6, coeffs);
    double bestDuration;
    double bestRoot;
    double root;
    double duration;
    bool found = false;
	Complex c;
    // iterate through all real roots to find the best valid solution
    for (int i=0; i<p.getDegree(); i++) {
        // get the next root
        c = p.getRoot(i);
        // check if its a real number
        if (c.i != 0) continue;
        root = c.r;
        
        // calc the time intervalls for this root and check if they are valid
        calcTimeIntervalls(t, type, bCruise, bDoubleDec, bDecAcc, bMoveForward, root, x0,
                xTarget, v0, vmax, a0, amax, jmax, stretchToTime);
        if (!areValidTimeIntervalls(t)) continue;
        
        // we got a valid solution --> remember root and duration if it is the
        // best solution we found so far
        duration = t[1]+t[2]+t[3]+t[4]+t[5]+t[6]+t[7];
        if ((!found) || (duration < bestDuration)) {
            bestRoot = root;
            bestDuration = duration;
            found = true;
            // in case this is a stretched profile we already got the solution
            if ((stretchToTime != 0) && isZero(stretchToTime-duration)) break;
         }
    }
    // throw an exception in case we found no solution
    if (!found) throw logic_error("No solution found for 3rd order trajectory of type " + type + ".");
    
    // otherwise give back the best solution
    root = bestRoot;
    calcTimeIntervalls(t, type, bCruise, bDoubleDec, bDecAcc, bMoveForward, root, x0,
                xTarget, v0, vmax, a0, amax, jmax, stretchToTime);
}

bool Stp7Formulars::areValidTimeIntervalls(double t[8]) {
    for (int i = 1; i < 8; i++) if (t[i] < 0) return false;
    return true;
}

/**
 * Call the appropriate specialist method for caluclate the coefficients of the
 * polynomial, which roots are used to calculate the solutions for the time intervalls.
 */
void Stp7Formulars::calcCoeffs(double coeffs[7],
        string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double stretchToTime,
        double t[8]) {
    double da = bDecAcc ? -1 : 1;
    double dc = bMoveForward ? 1 : -1;
    // stretched profile?
    if (stretchToTime == 0) {
        // double deceleration?
        if (bDoubleDec) {
            // case distinction for profile type
            if (type == Stp7::PROFILE_TT || type == Stp7::PROFILE_WT) {
                // double dec ?T profile
                //cout << "Coeffs for DoubleDec Canonical ?T" << endl;
                calcCoeffsDoubleDecProfile_T(coeffs,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc,t);
                return;
            } else if (type == Stp7::PROFILE_TW || type == Stp7::PROFILE_WW) {
                // double dec ?W profile
                //cout << "Coeffs for DoubleDec Canonical ?W" << endl;
                calcCoeffsDoubleDecProfile_W(coeffs,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc,t);
                return;
            }
        } else {
            // case distinction for profile type
            if (type == Stp7::PROFILE_TT) {
                // canonical TT profile
                //cout << "Coeffs for Fastest Canonical TT" << endl;
                calcCoeffsProfileTT(coeffs,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_WT) {
                // canonical WT profile
                //cout << "Coeffs for Fastest Canonical WT" << endl;
                calcCoeffsProfileWT(coeffs,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_TW) {
                // canonical TW profile
                //cout << "Coeffs for Fastest Canonical TW" << endl;
                calcCoeffsProfileTW(coeffs,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_WW) {
                // canonical WW profile
                //cout << "Coeffs for Fastest Canonical WW" << endl;
                calcCoeffsProfileWW(coeffs,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            }
        }
    } else { // stretched Profile
        // double deceleration?
        if (bDoubleDec) {
            // with cruising phase?
            if (bCruise) { // with cruising phase
                // case distinction for profile type
                if (type == Stp7::PROFILE_TT) {
                    // double dec TcT profile
                    //cout << "Coeffs for Stretched DoubleDec TcT" << endl;
                    calcCoeffsStretchedDoubleDecProfileTcT(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_WT) {
                    // double dec WcT profile
                    //cout << "Coeffs for Stretched DoubleDec WcT" << endl;
                    calcCoeffsStretchedDoubleDecProfileWcT(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_WW) {
                    // double dec WcW profile
                    //cout << "Coeffs for Stretched DoubleDec WcW" << endl;
                    calcCoeffsStretchedDoubleDecProfileWcW(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime); 
                    return;
                } else if (type == Stp7::PROFILE_TW) {
                    // double dec TcW profile
                    //cout << "Coeffs for Stretched DoubleDec TcW" << endl;
                    calcCoeffsStretchedDoubleDecProfileTcW(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime); 
                    return;
                }
            } else { // without cruising phase
                // case distinction for profile type
                if (type == Stp7::PROFILE_TT) {
                    // double dec TT profile
                    //cout << "Coeffs for Stretched DoubleDec TT" << endl;
                    calcCoeffsStretchedDoubleDecProfileTT(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime); 
                    return;
                } else if (type == Stp7::PROFILE_WT) {
                    // double dec WT profile
                    //cout << "Coeffs for Stretched DoubleDec WT" << endl;
                    calcCoeffsStretchedDoubleDecProfileWT(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime); 
                    return;
                } else if (type == Stp7::PROFILE_WW) {
                    // double dec WW profile
                    //cout << "Coeffs for Stretched DoubleDec WW" << endl;
                    calcCoeffsStretchedDoubleDecProfileWW(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime); 
                    return;
                } else if (type == Stp7::PROFILE_TW) {
                    // double dec TW profile
                    //cout << "Coeffs for Stretched DoubleDec TW" << endl;
                    calcCoeffsStretchedDoubleDecProfileTW(coeffs,x0,xTarget,v0,vmax,a0,amax,
                            jmax,da,dc,stretchToTime); 
                    return;
                }   
            }
        } else {
            // case distinction for profile type
            if (type == Stp7::PROFILE_TT) {
                // canonical TT profile
                //cout << "Coeffs for Stretched Canonical TT" << endl;
                calcCoeffsStretchedProfileTT(coeffs,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            } else if (type == Stp7::PROFILE_WT) {
                // canonical WT profile
                //cout << "Coeffs for Stretched Canonical WT" << endl;
                calcCoeffsStretchedProfileWT(coeffs,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            } else if (type == Stp7::PROFILE_TW) {
                // canonical TW profile
                //cout << "Coeffs for Stretched Canonical TW" << endl;
                calcCoeffsStretchedProfileTW(coeffs,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            } else if (type == Stp7::PROFILE_WW) {
                // canonical WW profile
                //cout << "Coeffs for Stretched Canonical WW" << endl;
                calcCoeffsStretchedProfileWW(coeffs,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            }
        }    
    }
    // unknown profile type, throw exception
    throw logic_error("Unknown profile: " + type + (bCruise ? ", " : ", no ") + "cruise.");
}

/**
 * Call the appropriate specialist method for caluclate the solution for the 
 * time intervalls using the passed value of the root of the polynomial.
 */
void Stp7Formulars::calcTimeIntervalls(double t[8],
        string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
        double root, double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double stretchToTime) {
    double da = bDecAcc ? -1 : 1;
    double dc = bMoveForward ? 1 : -1;    
    // stretched profile?
    if (stretchToTime == 0) {
        // double deceleration?
        if (bDoubleDec) {
            // case distinction for profile type
            if (type == Stp7::PROFILE_TT || type == Stp7::PROFILE_WT) {
                // double dec ?T profile
                //cout << "Times for DoubleDec Canonical ?T" << endl;
                calcTimeIntervallsDoubleDecProfile_T(t,root,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_TW || type == Stp7::PROFILE_WW) {
                // double dec ?W profile
                //cout << "Times for DoubleDec Canonical ?W" << endl;
                calcTimeIntervallsDoubleDecProfile_W(t,root,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            }
        } else {
            // case distinction for profile type
            if (type == Stp7::PROFILE_TT) {
                // canonical TT profile
                //cout << "Times for Fastest Canonical TT" << endl;
                calcTimeIntervallsProfileTT(t,root,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_WT) {
                // canonical WT profile
                //cout << "Times for Fastest Canonical TT" << endl;
                calcTimeIntervallsProfileWT(t,root,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_TW) {
                // canonical TW profile
                //cout << "Times for Fastest Canonical TW" << endl;
                calcTimeIntervallsProfileTW(t,root,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            } else if (type == Stp7::PROFILE_WW) {
                // canonical WW profile
                //cout << "Times for Fastest Canonical WW" << endl;
                calcTimeIntervallsProfileWW(t,root,x0,xTarget,v0,vmax,a0,amax,jmax,da,dc);
                return;
            }
        }
    } else { // stretched Profile
        // double deceleration?
        if (bDoubleDec) {
            // with cruising phase?
            if (bCruise) { // with cruising phase
                // case distinction for profile type
                if (type == Stp7::PROFILE_TT) {
                    // double dec TcT profile
                    //cout << "Times for Stretched DoubleDec TcT" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileTcT(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_WT) {
                    // double dec WcT profile
                    //cout << "Times for Stretched DoubleDec WcT" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileWcT(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_WW) {
                    // double dec WcW profile
                    //cout << "Times for Stretched DoubleDec WcW" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileWcW(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_TW) {
                    // double dec TcW profile
                    //cout << "Times for Stretched DoubleDec TcW" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileTcW(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                }
            } else { // without cruising phase
                // case distinction for profile type
                if (type == Stp7::PROFILE_TT) {
                    // double dec TT profile
                    //cout << "Times for Stretched DoubleDec TT" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileTT(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_WT) {
                    // double dec WT profile
                    //cout << "Times for Stretched DoubleDec WT" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileWT(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_WW) {
                    // double dec WW profile
                    //cout << "Times for Stretched DoubleDec WW" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileWW(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                } else if (type == Stp7::PROFILE_TW) {
                    // double dec TW profile
                    //cout << "Times for Stretched DoubleDec TW" << endl;
                    calcTimeIntervallsStretchedDoubleDecProfileTW(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                    return;
                }   
            }
        } else {
            // case distinction for profile type
            if (type == Stp7::PROFILE_TT) {
                // canonical TT profile
                //cout << "Times for Stretched Canonical TT" << endl;
                calcTimeIntervallsStretchedProfileTT(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            } else if (type == Stp7::PROFILE_WT) {
                // canonical WT profile
                //cout << "Times for Stretched Canonical WT" << endl;
                calcTimeIntervallsStretchedProfileWT(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            } else if (type == Stp7::PROFILE_TW) {
                // canonical TW profile
                //cout << "Times for Stretched Canonical TW" << endl;
                calcTimeIntervallsStretchedProfileTW(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            } else if (type == Stp7::PROFILE_WW) {
                // canonical WW profile
                //cout << "Times for Stretched Canonical WW" << endl;
                calcTimeIntervallsStretchedProfileWW(t,root,x0,xTarget,v0,vmax,a0,amax,
                        jmax,da,dc,stretchToTime);
                return;
            }
        }    
    }
    // unknown profile type, throw exception
    throw logic_error("Unknown profile: " + type + (bCruise ? ", " : ", no ") + "cruise.");
}

void Stp7Formulars::calcCoeffsProfileTT(double coeffs[7], double x0, double xTarget,
        double v0, double vmax, double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double amax2 = amax * amax;
    double t4 = jmax * v0;
    double t5 = da * dc;
    double t10 = amax2 * da;
    double t14 = amax2 * amax2;
    double t33 = a2 * a2;
    double t35 = jmax * jmax;
    double t36 = t35 * amax;
    double t43 = v0 * v0;
    coeffs[4] = 0.;
    coeffs[3] = 0.;
    coeffs[2] = 0.24e2;
    coeffs[1] = -0.24e2 * a2 + 0.48e2 * t4 * t5 + 0.24e2 * amax2 + 0.48e2 * t10;
    coeffs[0] = 0.24e2 * t14 * da + 0.24e2 * t14 + 0.48e2 * jmax * amax2 * dc * v0
                - 0.24e2 * t4 * da * a0 * amax - 0.12e2 * t4 * t5 * a2
                + 0.8e1 * a2 * a0 * dc * amax + 0.3e1 * t33
                - 0.24e2 * t36 * dc * xTarget + 0.24e2 * t36 * dc * x0
                + 0.12e2 * t35 * t43 + 0.36e2 * amax2 * v0 * jmax * da * dc
                - 0.18e2 * amax2 * a2 - 0.24e2 * t10 * a2;
}

void Stp7Formulars::calcTimeIntervallsProfileTT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double amax2 = amax * amax;
    t[1] = 0.1e1 / da / dc / jmax * (dc * amax - a0);
    t[2] = root / amax / jmax / da;
    t[3] = 0.1e1 / jmax * amax;
    t[4] = 0;
    t[5] = t[3];
    t[6] = (-amax2 * da + 0.2e1 * root + 0.2e1 * jmax * v0 * da * dc
            - a2 + amax2) / amax / jmax / da / 0.2e1;
    t[7] = t[3];    
}

void Stp7Formulars::calcCoeffsProfileWW(double coeffs[7], double x0, double xTarget,
        double v0, double vmax, double a0, double amax, double jmax, double da, double dc) {
    double amax2 = amax * amax;
    double jmax2 = jmax * jmax;
    double a2 = a0*a0;
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

void Stp7Formulars::calcTimeIntervallsProfileWW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double jmax2 = jmax * jmax;
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

void Stp7Formulars::calcCoeffsProfileWT(double coeffs[7], double x0, double xTarget,
        double v0, double vmax, double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double amax2 = amax * amax;
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

void Stp7Formulars::calcTimeIntervallsProfileWT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double amax2 = amax * amax;
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

void Stp7Formulars::calcCoeffsProfileTW(double coeffs[7], double x0, double xTarget,
        double v0, double vmax, double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double amax2 = amax * amax;
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

void Stp7Formulars::calcTimeIntervallsProfileTW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc) {
    double a2 = a0 * a0;
    double amax2 = amax * amax;  
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

/**
 * t[1] and t[2] must be set to correct values already.
 */
void Stp7Formulars::calcCoeffsDoubleDecProfile_T(double coeffs[7], double x0, double xTarget,
        double v0, double vmax, double a0, double amax, double jmax, double da, double dc, double t[8]) {
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

void Stp7Formulars::calcTimeIntervallsDoubleDecProfile_T(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc) {
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
 * t[1] and t[2] must be set to correct values already.
 */
void Stp7Formulars::calcCoeffsDoubleDecProfile_W(double coeffs[7], double x0, double xTarget,
        double v0, double vmax, double a0, double amax, double jmax, double da, double dc, double t[8]) {
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

void Stp7Formulars::calcTimeIntervallsDoubleDecProfile_W(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc) {
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

void Stp7Formulars::calcCoeffsStretchedProfileTT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t6 = amax * amax;
    double t7 = t6 * da;
    double t13 = a0 * a0;
    double t15 = jmax * v0;
    double t16 = da * dc;
    double t22 = t6 * amax;
    double t26 = dc * v0;
    double t37 = t6 * t6;
    double t44 = t13 * t13;
    double t50 = jmax * jmax;
    double t51 = v0 * v0;
    double t58 = amax * t50;
    
    coeffs[4] = 0;
    coeffs[3] = 0;
    coeffs[2] = 0.24e2;
    coeffs[1] = -0.24e2 * amax * jmax * da * stretchToTime + 0.60e2 * t7 + 0.12e2 * t6
            - 0.24e2 * dc * amax * a0 + 0.12e2 * t13 - 0.24e2 * t15 * t16;
    coeffs[0] = - 0.24e2 * stretchToTime * jmax * t22 - 0.24e2 * t6 * jmax * t26
            + 0.12e2 * t7 * t13 + 0.6e1 * t6 * t13 + 0.24e2 * t15 * da * a0 * amax
            + 0.12e2 * t37 * da - 0.8e1 * t13 * a0 * dc * amax + 0.3e1 * t44
            + 0.36e2 * t37 - 0.12e2 * t15 * t16 * t13 + 0.12e2 * t50 * t51
            - 0.24e2 * t22 * da * dc * a0 - 0.24e2 * t58 * dc * x0 + 0.24e2 * t58 * dc * xTarget
            - 0.12e2 * t7 * t26 * jmax;
}

void Stp7Formulars::calcTimeIntervallsStretchedProfileTT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    t[1] = 0.1e1 / da / dc / jmax * (dc * amax - a0);
    double t1 = amax * amax;
    double t8 = a0 * a0;
    t[2] = (t1 * da + 0.2e1 * root - 0.2e1 * jmax * v0 * da * dc + t8 - t1)
            / amax / jmax / da / 0.2e1;
    t[3] = 0.1e1 / jmax * amax;
    t[4] = (-t1 - 0.4e1 * root - t8 + 0.2e1 * v0 * jmax * da * dc
            + 0.2e1 * a0 * dc * amax + (-0.7e1 * t1 + 0.2e1 * amax * jmax
            * stretchToTime) * da) / amax / jmax / da / 0.2e1;
    t[5] = 0.1e1 / jmax * amax;
    t[6] = root / amax / jmax / da;
    t[7] = 0.1e1 / jmax * amax;
}

void Stp7Formulars::calcCoeffsStretchedProfileWW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t1 = jmax * jmax;
    double t2 = a0 * a0;
    double t3 = t2 * a0;
    double t4 = t1 * t3;
    double t9 = t1 * t2;
    double t10 = v0 * v0;
    double t13 = t1 * jmax;
    double t14 = t13 * da;
    double t15 = t14 * dc;
    double t16 = v0 * a0;
    double t17 = t16 * xTarget;
    double t20 = t16 * x0;
    double t23 = t13 * dc;
    double t33 = a0 * stretchToTime;
    double t42 = t2 * t2;
    double t45 = t1 * da;
    double t55 = t1 * a0;
    double t60 = dc * jmax;
    double t61 = da * t3;
    double t70 = t2 * da;
    double t83 = -0.72e2 * t14 * dc * x0 * stretchToTime + 0.72e2 * t1 * v0 * t33 + 0.96e2 * t1 * t10 + 0.72e2 * t14 * dc * stretchToTime * xTarget + 0.6e1 * da * t42 + 0.72e2 * t45 * a0 * xTarget + 0.72e2 * t23 * stretchToTime * xTarget - 0.72e2 * t45 * a0 * x0 - 0.72e2 * t55 * x0 + 0.72e2 * t55 * xTarget - 0.24e2 * t60 * t61 * stretchToTime + 0.120e3 * t45 * t10 - 0.48e2 * t60 * t2 * v0 - 0.24e2 * t60 * t70 * v0 - 0.24e2 * t60 * t3 * stretchToTime - 0.72e2 * t23 * x0 * stretchToTime + 0.72e2 * t45 * t16 * stretchToTime;
    double t85 = v0 * dc;
    double t86 = jmax * a0;
    double t104 = t1 * t1;
    double t105 = t104 * da;
    double t106 = x0 * x0;
    double t109 = t10 * v0;
    double t113 = 0.24e2 * t4 * xTarget - 0.24e2 * t4 * x0 + 0.24e2 * t9 * t10 - 0.72e2 * t15 * t17 + 0.72e2 * t15 * t20 - 0.72e2 * t23 * t17 + 0.72e2 * t23 * t20 - 0.36e2 * t105 * t106 - 0.32e2 * t23 * t109 + t42 * t2;
    double t117 = xTarget * xTarget;
    double t135 = da * dc;
    double t139 = a0 * da;
    double t155 = stretchToTime * stretchToTime;
    double t173 = 0.72e2 * t104 * x0 * xTarget - 0.36e2 * t105 * t117 - 0.24e2 * t45 * t3 * x0 + 0.24e2 * t45 * t3 * xTarget - 0.40e2 * t14 * dc * t109 + 0.12e2 * t9 * da * t10 + 0.72e2 * t105 * x0 * xTarget - 0.36e2 * t104 * t106 - 0.6e1 * t60 * da * v0 * t42 - 0.36e2 * t104 * t117;
    
    coeffs[6] = 0.4e1 * da - 0.4e1;
    coeffs[5] = 0.72e2 * t135 * jmax * stretchToTime + 0.72e2 * a0 + 0.72e2 * t60 * stretchToTime + 0.72e2 * t139;
    coeffs[4] = -0.72e2 * t60 * t33 - 0.96e2 * t85 * jmax - 0.36e2 * t1 * t155 - 0.120e3 * t135 * jmax * v0 - 0.72e2 * t60 * t139 * stretchToTime + 0.24e2 * t2 - 0.36e2 * t45 * t155 + 0.12e2 * t70;
    coeffs[3] = -0.72e2 * t85 * t86 * da - 0.72e2 * t85 * t86 + 0.24e2 * t3 + 0.72e2 * t1 * x0 + 0.24e2 * t61 + 0.72e2 * t45 * x0 - 0.72e2 * t45 * xTarget - 0.72e2 * t1 * xTarget;
    coeffs[2] = t83;
    coeffs[1] = 0;
    coeffs[0] = t113 + t173;
}

void Stp7Formulars::calcTimeIntervallsStretchedProfileWW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double dFrac;
    {
        double t1 = jmax * jmax;
        double t4 = pow(root, 0.2e1);
        double t5 = jmax * t4;
        double t7 = a0 * a0;
        double t8 = t7 * jmax;
        dFrac = 0.1e1 / (0.2e1 * t1 * v0 + (-0.4e1 * t5 - t8) * da * dc + (-0.2e1 * t5 - 0.2e1 * t8) * dc + 0.4e1 * t1 * da * v0);
    }
    
    double t1 = pow(root, 0.2e1);
    double t3 = 0.2e1 * t1 * a0;
    double t4 = a0 * a0;
    double t21 = jmax * jmax;
    t[1] = (-t3 + t4 * a0 + (-0.6e1 * jmax * t1 * stretchToTime - 0.2e1 * v0 * jmax * a0) * da * dc + 0.2e1 * v0 * dc * jmax * a0 + (0.6e1 * t1 * root + t3 - 0.6e1 * t21 * x0 + 0.6e1 * t21 * xTarget) * da) * dFrac;
    t[2] = 0.;
    t[3] = (-0.6e1 * t1 * a0 * da - 0.2e1 * t4 * a0 + 0.6e1 * v0 * dc * jmax * a0 * da + 0.6e1 * t1 * root - 0.6e1 * dc * jmax * t1 * stretchToTime - 0.6e1 * t21 * x0 + 0.6e1 * t21 * xTarget) * dFrac;
    {
        double t1 = a0 * a0;
        double t2 = t1 * root;
        double t4 = jmax * jmax;
        double t6 = v0 * t4 * stretchToTime;
        double t9 = 0.6e1 * t4 * xTarget;
        double t11 = 0.6e1 * t4 * x0;
        double t12 = pow(root, 0.2e1);
        double t14 = 0.2e1 * t12 * root;
        double t15 = t12 * a0;
        double t19 = jmax * t1 * stretchToTime;
        double t21 = v0 * jmax * a0;
        double t24 = jmax * t12 * stretchToTime;
        double t27 = root * jmax * v0;
        t[4] = (0.4e1 * t2 + 0.2e1 * t6 - t9 + t11 - t14 + 0.2e1 * t15 + t1 * a0 + (-t19 - 0.4e1 * t21 + 0.2e1 * t24 - 0.8e1 * t27) * da * dc + (-0.4e1 * t27 - 0.2e1 * t21 - 0.2e1 * t19 + 0.4e1 * t24) * dc + (-t9 + t11 + 0.2e1 * t2 + 0.4e1 * t15 + t14 + 0.4e1 * t6) * da) * dFrac;
    }
    t[5] = root / dc / jmax;
    t[6] = 0;
    t[7] = t[5];
}

void Stp7Formulars::calcCoeffsStretchedProfileTW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t5 = a0 * a0;
    double t8 = dc * da;
    double t18 = amax * amax;
    double t19 = t18 * da;
    double t23 = v0 * jmax;
    double t27 = jmax * jmax;
    double t28 = t27 * amax;
    double t41 = t5 * t5;
    double t50 = v0 * v0;

    coeffs[4] = 0.12e2;
    coeffs[3] = 0.24e2 * amax * dc;
    coeffs[2] = (0.12e2 * da * t5 - 0.24e2 * t8 * a0 * amax - 0.24e2 * jmax * stretchToTime * amax - 0.24e2 * dc * jmax * v0 + 0.12e2 * t19);
    coeffs[1] = 0;
    coeffs[0] = -0.12e2 * t23 * t8 * t5 - 0.24e2 * t28 * dc * x0 - 0.12e2 * t23 * t19 * dc + 0.6e1 * t5 * t18 + 0.24e2 * amax * da * t23 * a0 + 0.3e1 * t41 - 0.8e1 * t5 * a0 * dc * amax + 0.24e2 * t28 * dc * xTarget + 0.12e2 * t50 * t27;
}

void Stp7Formulars::calcTimeIntervallsStretchedProfileTW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    t[1] = (dc * amax - a0) / da / dc / jmax;
    double t1 = a0 * a0;
    double t2 = amax * amax;
    double t7 = pow(root, 0.2e1);
    t[2] = (t1 - t2 - 0.2e1 * v0 * jmax * da * dc + (0.2e1 * t7 - t2) * da) / amax / jmax / da / 0.2e1;
    t[3] = amax / jmax;
    double t20 = pow(root, 0.2e1);
    t[4] = (-t2 - t1 + (0.2e1 * v0 * jmax - 0.4e1 * root * amax) * da * dc + (-0.2e1 * t20 - t2 + 0.2e1 * jmax * stretchToTime * amax) * da + 0.2e1 * a0 * dc * amax) / amax / jmax / da / 0.2e1;
    t[5] = root / dc / jmax;
    t[6] = 0;
    t[7] = t[5];
}

void Stp7Formulars::calcCoeffsStretchedProfileWT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t8 = dc * amax;
    double t10 = a0 * da;
    double t14 = dc * jmax;
    double t21 = amax * amax;
    double t23 = a0 * a0;
    double t28 = jmax * stretchToTime;
    double t29 = da * amax;
    double t33 = dc * a0;
    double t53 = t23 * a0;
    double t60 = t21 * a0;
    double t73 = jmax * jmax;
    double t74 = v0 * v0;
    double t85 = t73 * amax;
    double t90 = t23 * t23;

    coeffs[4] = 0.6e1 * da + 0.6e1;
    coeffs[3] = 0.12e2 * dc * da * amax + 0.24e2 * a0 + 0.12e2 * t8 + 0.24e2 * t10;
    coeffs[2] = 0.12e2 * t14 * v0 * da + 0.12e2 * v0 * jmax * dc + 0.6e1 * t21 + 0.30e2 * t23 * da + 0.6e1 * t21 * da - 0.12e2 * t28 * t29 + 0.30e2 * t23 + 0.24e2 * t33 * amax - 0.12e2 * t28 * amax + 0.24e2 * t33 * t29;
    coeffs[1] = 0.24e2 * t14 * v0 * a0 + 0.24e2 * t14 * t10 * v0 + 0.12e2 * dc * t23 * t29 + 0.12e2 * t8 * t23 + 0.12e2 * t53 * da + 0.12e2 * t53 - 0.24e2 * t28 * t10 * amax + 0.12e2 * t60 + 0.12e2 * t60 * da - 0.24e2 * t28 * a0 * amax;
    coeffs[0] = 0.12e2 * jmax * t21 * dc * v0 + 0.12e2 * t73 * t74 - 0.12e2 * t28 * t23 * amax + 0.4e1 * t53 * dc * amax + 0.6e1 * t21 * t23 - 0.24e2 * t85 * dc * stretchToTime * v0 + 0.3e1 * t90 + 0.24e2 * t85 * dc * xTarget - 0.24e2 * t85 * dc * x0 + 0.12e2 * t23 * v0 * t14;
}

void Stp7Formulars::calcTimeIntervallsStretchedProfileWT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    t[1] = root / dc / jmax;
    t[2] = 0;
    t[3] = (a0 + da * root) / dc / jmax;
    double t1 = amax * amax;
    double t3 = a0 * a0;
    double t4 = pow(root, 0.2e1);
    double t6 = 0.2e1 * root * a0;
    t[4] = (-0.2e1 * t1 - t3 - t4 - t6 + 0.2e1 * stretchToTime * jmax * amax + (-0.2e1 * root * amax - 0.2e1 * a0 * amax - 0.2e1 * v0 * jmax) * dc + (-t6 - t4) * da - 0.2e1 * dc * amax * da * root) / amax / jmax / 0.2e1;
    t[5] = amax / jmax;
    t[6] = (t3 + t4 - 0.2e1 * t1 + t6 + 0.2e1 * v0 * jmax * dc + (t6 + t4) * da) / amax / jmax / 0.2e1;
    t[7] = t[5];
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileWcW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t1 = jmax * jmax;
    double t2 = t1 * da;
    double t3 = a0 * a0;
    double t4 = t3 * a0;
    double t8 = t1 * jmax;
    double t10 = t8 * v0 * dc;
    double t11 = a0 * da;
    double t15 = da * dc;
    double t16 = v0 * v0;
    double t17 = t16 * v0;
    double t27 = t1 * t1;
    double t28 = da * t27;
    double t32 = t3 * t3;
    double t34 = t8 * dc;
    double t35 = v0 * a0;
    double t39 = x0 * x0;
    double t42 = dc * jmax;
    double t53 = t1 * a0;
    double t83 = da * t3;
    double t92 = da * t4;
    double t99 = a0 * stretchToTime;
    double t102 = 0.72e2 * t34 * x0 * stretchToTime + 0.72e2 * t53 * xTarget + 0.72e2 * t2 * t35 * stretchToTime + 0.72e2 * t15 * t8 * x0 * stretchToTime + 0.72e2 * t2 * a0 * xTarget - 0.72e2 * t15 * t8 * stretchToTime * xTarget + 0.96e2 * t1 * t16 + 0.120e3 * t2 * t16 + 0.48e2 * t42 * t3 * v0 - 0.72e2 * t2 * a0 * x0 - 0.72e2 * t34 * stretchToTime * xTarget + 0.24e2 * t42 * t83 * v0 + 0.24e2 * t42 * t4 * stretchToTime + 0.6e1 * da * t32 + 0.24e2 * t42 * t92 * stretchToTime - 0.72e2 * t53 * x0 + 0.72e2 * t1 * v0 * t99;
    double t104 = 0.24e2 * t2 * t4 * x0 + 0.72e2 * t10 * t11 * x0 - 0.40e2 * t15 * t8 * t17 - 0.24e2 * t2 * t4 * xTarget - 0.12e2 * t2 * t3 * t16 - 0.72e2 * t28 * x0 * xTarget - t32 * t3 - 0.72e2 * t34 * t35 * xTarget + 0.36e2 * t27 * t39 - 0.6e1 * t42 * da * v0 * t32 - 0.72e2 * t10 * t11 * xTarget;
    double t111 = stretchToTime * stretchToTime;
    double t114 = v0 * dc;
    double t128 = xTarget * xTarget;
    double t131 = jmax * a0;
    double t152 = t1 * t4;
    double t175 = 0.72e2 * t34 * t35 * x0 + 0.36e2 * t27 * t128 - 0.24e2 * t152 * xTarget + 0.24e2 * t152 * x0 + 0.36e2 * t28 * t128 + 0.36e2 * t28 * t39 - 0.72e2 * t27 * x0 * xTarget - 0.24e2 * t1 * t3 * t16 - 0.32e2 * t34 * t17;
    
    coeffs[6] = 0.68e2 + 0.76e2 * da;
    coeffs[5] = 0.72e2 * a0 - 0.72e2 * t42 * stretchToTime + 0.72e2 * t11 - 0.72e2 * t42 * da * stretchToTime;
    coeffs[4] = -0.72e2 * t42 * t99 - 0.24e2 * t3 + 0.36e2 * t1 * t111 - 0.96e2 * t114 * jmax + 0.36e2 * t2 * t111 - 0.12e2 * t83 - 0.120e3 * t15 * jmax * v0 - 0.72e2 * t42 * t11 * stretchToTime;
    coeffs[3] = -0.72e2 * t114 * t131 * da - 0.24e2 * t92 - 0.24e2 * t4 + 0.72e2 * t1 * xTarget - 0.72e2 * t2 * x0 - 0.72e2 * t114 * t131 + 0.72e2 * t2 * xTarget - 0.72e2 * t1 * x0;
    coeffs[2] = t102;
    coeffs[1] = 0.;
    coeffs[0] = t104 + t175;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileWcW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double frac;
    {
        double t1 = jmax * jmax;
        double t2 = t1 * v0;
        double t6 = pow(root, 0.2e1);
        double t7 = jmax * t6;
        double t8 = a0 * a0;
        double t9 = t8 * jmax;
        frac = 0.1e1 / (0.2e1 * t2 + 0.4e1 * t2 * da + (-0.2e1 * t7 + 0.2e1 * t9) * dc + (-0.4e1 * t7 + t9) * dc * da);
    }
    {
        double t1 = pow(root, 0.2e1);
        double t3 = 0.2e1 * t1 * a0;
        double t4 = a0 * a0;
        double t8 = jmax * jmax;
        t[1] = (t3 + t4 * a0 + (0.6e1 * t1 * root - t3 - 0.6e1 * t8 * x0 + 0.6e1 * t8 * xTarget) * da - 0.2e1 * v0 * dc * jmax * a0 + (-0.6e1 * jmax * t1 * stretchToTime + 0.2e1 * v0 * jmax * a0) * dc * da) * frac;
    }
    t[2] = 0.;
    {
        double t1 = pow(root, 0.2e1);
        double t5 = a0 * a0;
        double t19 = jmax * jmax;
        t[3] = (0.6e1 * t1 * a0 * da - 0.2e1 * t5 * a0 - 0.6e1 * v0 * dc * jmax * a0 * da + 0.6e1 * t1 * root - 0.6e1 * dc * jmax * t1 * stretchToTime - 0.6e1 * t19 * x0 + 0.6e1 * t19 * xTarget) * frac;
    }
    {
        double t1 = a0 * a0;
        double t3 = jmax * jmax;
        double t5 = 0.6e1 * t3 * x0;
        double t7 = 0.6e1 * t3 * xTarget;
        double t9 = t3 * stretchToTime * v0;
        double t11 = pow(root, 0.2e1);
        double t13 = 0.2e1 * t11 * root;
        double t14 = t11 * a0;
        double t16 = t1 * root;
        double t24 = v0 * jmax * a0;
        double t27 = jmax * t11 * stretchToTime;
        double t30 = t1 * jmax * stretchToTime;
        double t33 = root * jmax * v0;
        t[4] = (t1 * a0 + t5 - t7 + 0.2e1 * t9 - t13 - 0.2e1 * t14 - 0.4e1 * t16 + (t5 - t7 + t13 - 0.2e1 * t16 + 0.4e1 * t9 - 0.4e1 * t14) * da + (0.2e1 * t24 + 0.4e1 * t27 + 0.2e1 * t30 - 0.4e1 * t33) * dc + (-0.8e1 * t33 + 0.4e1 * t24 + t30 + 0.2e1 * t27) * dc * da) * frac;
    }
    t[5] = root / dc / jmax;
    t[6] = 0.;
    t[7] = t[5];
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileWW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t1 = a0 * a0;
    double t4 = dc * jmax;
    double t12 = jmax * jmax;
    double t13 = stretchToTime * stretchToTime;
    double t14 = t12 * t13;
    double t19 = v0 * dc;
    double t27 = xTarget * t12;
    double t31 = x0 * t12;
    double t33 = t12 * jmax;
    double t35 = t33 * t13 * stretchToTime;
    double t42 = t1 * a0;
    double t47 = stretchToTime * t1;
    double t58 = -0.96e2 * t27 - 0.96e2 * t27 * da + 0.96e2 * t31 + 0.12e2 * t35 * dc - 0.12e2 * t14 * a0 + 0.96e2 * t31 * da - 0.4e1 * t42 - 0.12e2 * t14 * a0 * da - 0.12e2 * t4 * t47 * da - 0.4e1 * t42 * da + 0.12e2 * t35 * dc * da - 0.12e2 * t4 * t47;
    double t64 = t33 * stretchToTime;
    double t70 = t12 * t12;
    double t71 = t13 * t13;
    double t77 = v0 * v0;
    double t83 = t1 * t1;
    
    coeffs[2] = 0.12e2 * t1 * da + 0.24e2 * t4 * stretchToTime * da * a0 + 0.24e2 * t4 * stretchToTime * a0 - 0.12e2 * t14 - 0.12e2 * t14 * da + 0.12e2 * t1 + 0.48e2 * t19 * jmax + 0.48e2 * t19 * jmax * da;
    coeffs[1] = t58;
    coeffs[0] = 0.6e1 * t14 * t1 + 0.96e2 * t27 * a0 - 0.96e2 * t64 * dc * x0 - 0.96e2 * t31 * a0 - 0.3e1 * t70 * t71 + 0.4e1 * t4 * stretchToTime * t42 + 0.48e2 * t77 * t12 - 0.48e2 * t33 * t13 * t19 + t83 + 0.96e2 * t64 * dc * xTarget - 0.12e2 * t35 * dc * a0;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileWW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double frac;
    {
        double t1 = jmax * jmax;
        frac = 0.1e1 / (0.4e1 * stretchToTime * t1 - 0.4e1 * dc * jmax * da * root + (-0.4e1 * jmax * root + 0.4e1 * jmax * a0) * dc);
    }
    t[1] = root / dc / jmax;
    t[2] = 0.;
    {
        double t3 = jmax * jmax;
        double t4 = stretchToTime * stretchToTime;
        double t6 = a0 * a0;
        double t12 = jmax * stretchToTime;
        t[3] = (-0.2e1 * root * a0 + t3 * t4 - t6 + 0.2e1 * dc * jmax * stretchToTime * da * root + (-0.2e1 * t12 * root - 0.2e1 * t12 * a0 - 0.4e1 * v0 * jmax) * dc + 0.2e1 * a0 * da * root) * frac;
    }
    t[4] = 0.;
    t[5] = (-root + dc * jmax * stretchToTime + a0 - da * root) / dc / jmax / 0.2e1;
    t[6] = 0.;
    {
        double t3 = jmax * jmax;
        double t4 = stretchToTime * stretchToTime;
        double t6 = a0 * a0;
        double t14 = jmax * stretchToTime;
        t[7] = (0.2e1 * root * a0 + t3 * t4 - t6 - 0.2e1 * dc * jmax * stretchToTime * da * root + (0.4e1 * v0 * jmax - 0.2e1 * t14 * root + 0.2e1 * t14 * a0) * dc + 0.2e1 * a0 * da * root) * frac;
    }
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileTcW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t8 = amax * amax;
    double t11 = a0 * a0;
    double t14 = dc * da;
    double t23 = v0 * jmax;
    double t27 = jmax * jmax;
    double t28 = amax * t27;
    double t47 = t11 * t11;
    double t51 = v0 * v0;

    coeffs[4] = 0.12e2;
    coeffs[3] = - 0.24e2* dc * amax;
    coeffs[2] = -0.24e2 * dc * v0 * jmax - 0.12e2 * t8 * da - 0.12e2 * da * t11 - 0.24e2 * t14 * a0 * amax + 0.24e2 * stretchToTime * amax * jmax;
    coeffs[1] = 0;
    coeffs[0] = 0.12e2 * t23 * t14 * t11 - 0.24e2 * t28 * dc * xTarget + 0.12e2 * jmax * t8 * t14 * v0 + 0.8e1 * t11 * a0 * dc * amax + 0.24e2 * t28 * dc * x0 + 0.24e2 * t23 * da * a0 * amax + 0.3e1 * t47 + 0.6e1 * t11 * t8 + 0.12e2 * t51 * t27;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileTcW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    t[1] = (dc * amax + a0) / da / dc / jmax;
    {
        double t1 = a0 * a0;
        double t2 = amax * amax;
        double t3 = pow(root, 0.2e1);
        t[2] = (t1 - t2 + (-0.2e1 * t3 - t2) * da + 0.2e1 * v0 * jmax * da * dc) / amax / jmax / da / 0.2e1;
    }
    t[3] = amax / jmax;
    {
        double t1 = amax * amax;
        double t2 = a0 * a0;
        double t6 = pow(root, 0.2e1);
        t[4] = (-t1 - t2 - 0.2e1 * dc * a0 * amax + (0.2e1 * t6 - t1 + 0.2e1 * stretchToTime * amax * jmax) * da + (-0.2e1 * v0 * jmax - 0.4e1 * root * amax) * dc * da) / amax / jmax / da / 0.2e1;
    }
    t[5] = root / dc / jmax;
    t[6] = 0;
    t[7] = t[5];
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileTW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t2 = amax * amax;
    double t3 = da * t2;
    double t5 = dc * da;
    double t6 = a0 * amax;
    double t9 = stretchToTime * jmax;
    double t12 = a0 * a0;
    double t16 = jmax * v0;
    double t21 = dc * amax;
    double t24 = t2 * amax;
    double t29 = dc * jmax;
    double t34 = dc * t24;
    double t36 = jmax * jmax;
    double t41 = amax * stretchToTime;
    double t42 = da * jmax;
    double t49 = t12 * a0;
    double t56 = t36 * stretchToTime * v0;
    double t58 = stretchToTime * stretchToTime;
    double t62 = dc * t2;
    double t71 = -0.24e2 * t21 * t12 - 0.24e2 * t5 * t24 - 0.24e2 * a0 * t2 + 0.24e2 * t29 * stretchToTime * da * t12 - 0.24e2 * t34 - 0.48e2 * xTarget * t36 + 0.48e2 * x0 * t36 + 0.48e2 * t41 * t42 * a0 - 0.48e2 * jmax * amax * v0 - 0.8e1 * t49 + 0.24e2 * t5 * t2 * stretchToTime * jmax + 0.48e2 * t56 - 0.24e2 * t21 * t58 * t36 + 0.48e2 * t62 * t9 - 0.24e2 * t5 * amax * t12 - 0.48e2 * da * a0 * t2;
    double t73 = t24 * stretchToTime;
    double t76 = t2 * t2;
    double t81 = v0 * da;
    double t90 = t12 * t12;
    double t94 = 0.12e2 * t73 * jmax - 0.6e1 * da * t76 + 0.12e2 * t73 * t42 - 0.24e2 * t6 * t81 * jmax - 0.12e2 * t21 * t49 - 0.12e2 * t81 * t29 * t12 - 0.3e1 * t90 - 0.12e2 * t34 * a0;
    double t105 = v0 * v0;
    double t125 = -0.6e1 * t3 * t12 - 0.12e2 * t2 * t58 * t36 - 0.6e1 * t76 - 0.12e2 * t5 * t2 * v0 * jmax - 0.12e2 * t105 * t36 - 0.12e2 * t5 * t24 * a0 + 0.24e2 * t5 * t2 * a0 * stretchToTime * jmax + 0.12e2 * t41 * t42 * t12 - 0.12e2 * t62 * t16 - 0.18e2 * t2 * t12 + 0.24e2 * t21 * t56;
    
    coeffs[4] = 0.12e2;
    coeffs[3] = 0;
    coeffs[2] = -0.24e2 * t3 - 0.48e2 * t5 * t6 + 0.48e2 * t9 * amax - 0.24e2 * da * t12 - 0.24e2 * t2 - 0.48e2 * t16 * dc;
    coeffs[1] = t71;
    coeffs[0] = t94 + t125;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileTW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    t[1] = (dc * amax + a0) / da / dc / jmax;
    t[2] = (-a0 - dc * amax - 0.2e1 * root * da + (-amax + stretchToTime * jmax) * dc * da) / da / dc / jmax;
    {
        double t1 = amax * amax;
        double t2 = a0 * a0;
        double t6 = pow(root, 0.2e1);
        t[3] = (-t1 - t2 - 0.2e1 * dc * amax * a0 + (-t1 + 0.2e1 * t6 + 0.2e1 * jmax * stretchToTime * amax) * da - 0.2e1 * v0 * da * dc * jmax) / root / dc / jmax / da / 0.4e1;
    }
    t[4] = 0;
    t[5] = root / dc / jmax;
    t[6] = 0;
    {
        double t1 = amax * amax;
        double t2 = a0 * a0;
        double t6 = pow(root, 0.2e1);
        t[7] = (t1 + t2 + 0.2e1 * dc * amax * a0 + (t1 + 0.2e1 * t6 - 0.2e1 * jmax * stretchToTime * amax) * da + (0.4e1 * amax * root + 0.2e1 * jmax * v0) * dc * da) / root / dc / jmax / da / 0.4e1;
    }
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileWcT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t5 = a0 * da;
    double t7 = dc * amax;
    double t14 = a0 * a0;
    double t17 = dc * a0;
    double t20 = amax * amax;
    double t21 = t20 * da;
    double t24 = jmax * stretchToTime;
    double t27 = amax * da;
    double t30 = v0 * jmax;
    double t36 = dc * jmax;
    double t45 = t14 * a0;
    double t69 = t14 * t14;
    double t71 = jmax * jmax;
    double t81 = v0 * v0;
    
    coeffs[4] = 0.6e1 * da + 0.6e1;
    coeffs[3] = -0.24e2 * a0 - 0.24e2 * t5 - 0.12e2 * t7 - 0.12e2 * dc * da * amax;
    coeffs[2] = 0.30e2 * t14 * da + 0.24e2 * t17 * amax - 0.6e1 * t21 + 0.30e2 * t14 + 0.12e2 * t24 * amax + 0.12e2 * t27 * t24 - 0.12e2 * t30 * dc + 0.24e2 * t17 * t27 - 0.6e1 * t20 - 0.12e2 * t36 * v0 * da;
    coeffs[1] = -0.12e2 * dc * t14 * t27 - 0.12e2 * t45 - 0.12e2 * t45 * da + 0.24e2 * t36 * t5 * v0 + 0.12e2 * t21 * a0 - 0.12e2 * t7 * t14 + 0.12e2 * t20 * a0 - 0.24e2 * t24 * a0 * amax + 0.24e2 * t36 * v0 * a0 - 0.24e2 * t27 * t24 * a0;
    coeffs[0] = 0.3e1 * t69 - 0.24e2 * t7 * t71 * stretchToTime * v0 - 0.12e2 * t14 * v0 * t36 - 0.6e1 * t14 * t20 + 0.12e2 * t71 * t81 + 0.12e2 * amax * t14 * t24 + 0.12e2 * dc * t20 * t30 - 0.24e2 * t7 * t71 * x0 + 0.24e2 * t7 * t71 * xTarget + 0.4e1 * t45 * dc * amax;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileWcT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    t[1] = root / dc / jmax;
    t[2] = 0;
    t[3] = (-a0 + da * root) / dc / jmax;
    {
        double t1 = amax * amax;
        double t3 = a0 * a0;
        double t4 = pow(root, 0.2e1);
        double t6 = 0.2e1 * root * a0;
        t[4] = (-0.2e1 * t1 + t3 + t4 - t6 + 0.2e1 * jmax * stretchToTime * amax + (-t6 + t4) * da + (-0.2e1 * root * amax + 0.2e1 * a0 * amax - 0.2e1 * v0 * jmax) * dc - 0.2e1 * dc * amax * da * root) / amax / jmax / 0.2e1;
    }
    t[5] = amax / jmax;
    {
        double t1 = a0 * a0;
        double t2 = pow(root, 0.2e1);
        double t3 = amax * amax;
        double t6 = 0.2e1 * root * a0;
        t[6] = (-t1 - t2 - 0.2e1 * t3 + t6 + (t6 - t2) * da + 0.2e1 * v0 * jmax * dc) / amax / jmax / 0.2e1;
    }
    t[7] = t[5];
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileWT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t1 = a0 * amax;
    double t2 = jmax * jmax;
    double t3 = t2 * jmax;
    double t4 = stretchToTime * t3;
    double t8 = dc * amax;
    double t9 = a0 * a0;
    double t10 = t8 * t9;
    double t11 = stretchToTime * t2;
    double t12 = t11 * v0;
    double t15 = t9 * t9;
    double t16 = amax * amax;
    double t19 = amax * stretchToTime;
    double t20 = v0 * v0;
    double t24 = t16 * a0;
    double t25 = x0 * t2;
    double t28 = amax * v0;
    double t29 = t3 * xTarget;
    double t32 = t9 * a0;
    double t33 = jmax * t32;
    double t36 = dc * xTarget;
    double t37 = t2 * amax;
    double t38 = t37 * t9;
    double t41 = a0 * v0;
    double t42 = t3 * amax;
    double t43 = stretchToTime * stretchToTime;
    double t47 = xTarget * t2;
    double t50 = t16 * t9;
    double t51 = t43 * t2;
    double t54 = x0 * x0;
    double t55 = t2 * t2;
    double t58 = -0.144e3 * t1 * t4 * x0 - 0.72e2 * t10 * t12 + 0.12e2 * t15 * t16 + 0.216e3 * t19 * t3 * t20 - 0.72e2 * t24 * t25 - 0.144e3 * t28 * t29 - 0.24e2 * t28 * t33 + 0.144e3 * t36 * t38 + 0.72e2 * t41 * t42 * t43 + 0.72e2 * t47 * t24 - 0.36e2 * t50 * t51 - 0.72e2 * t54 * t55;
    double t59 = t16 * amax;
    double t61 = jmax * a0;
    double t65 = dc * jmax;
    double t68 = t3 * x0;
    double t72 = stretchToTime * jmax;
    double t75 = t2 * a0;
    double t76 = t75 * t20;
    double t82 = dc * a0;
    double t83 = v0 * t3;
    double t84 = t83 * xTarget;
    double t87 = t16 * t16;
    double t89 = a0 * stretchToTime;
    double t90 = t89 * jmax;
    double t93 = a0 * da;
    double t96 = da * v0;
    double t99 = t9 * da;
    double t101 = t16 * da;
    double t109 = v0 * jmax;
    double t112 = amax * jmax;
    double t113 = stretchToTime * da;
    double t118 = dc * t32;
    double t120 = stretchToTime * t16 * jmax;
    double t123 = dc * t59;
    double t129 = t15 * a0;
    double t133 = 0.72e2 * t59 * v0 * t61 - 0.6e1 * t15 * v0 * t65 + 0.144e3 * t28 * t68 + 0.72e2 * t9 * t59 * t72 + 0.72e2 * t8 * t76 + 0.6e1 * t15 * amax * t72 - 0.144e3 * t82 * t84 - 0.36e2 * dc * t87 * t90 + 0.48e2 * t118 * t120 + 0.72e2 * t123 * t12 + 0.144e3 * xTarget * t55 * x0 + 0.6e1 * t129 * dc * amax;
    double t153 = dc * da;
    double t157 = t99 * v0;
    double t160 = dc * t2;
    double t166 = jmax * t9;
    double t175 = da * xTarget;
    double t178 = da * x0;
    double t184 = amax * da;
    double t187 = dc * t9;
    double t190 = jmax * t59;
    double t193 = t2 * da;
    double t194 = t16 * t43;
    double t202 = t153 * t2;
    double t203 = t19 * v0;
    double t207 = dc * t16;
    double t210 = 0.216e3 * t153 * t37 * xTarget - 0.36e2 * t65 * t157 + 0.216e3 * t160 * amax * xTarget - 0.72e2 * t112 * t41 + 0.36e2 * t166 * t19 + 0.36e2 * t153 * t59 * a0 - 0.216e3 * t160 * amax * x0 + 0.216e3 * t75 * t175 - 0.216e3 * t75 * t178 - 0.72e2 * t65 * t16 * v0 + 0.108e3 * t118 * t184 - 0.36e2 * t109 * t187 + 0.180e3 * t190 * t113 - 0.144e3 * t193 * t194 - 0.36e2 * t2 * t20 + 0.180e3 * t153 * t16 * t90 + 0.72e2 * t202 * t203 - 0.36e2 * t87 + 0.180e3 * t207 * t90;
    double t211 = jmax * t16;
    double t215 = t87 * da;
    double t217 = t1 * t43;
    double t231 = t93 * v0;
    double t245 = t184 * stretchToTime;
    double t254 = -0.72e2 * t153 * t211 * v0 - 0.36e2 * t215 - 0.108e3 * t160 * t217 + 0.144e3 * t50 + 0.27e2 * t15 + 0.27e2 * t15 * da - 0.216e3 * t153 * t37 * x0 - 0.108e3 * t202 * t217 + 0.72e2 * t160 * t203 - 0.72e2 * t112 * t231 + 0.108e3 * t8 * t32 + 0.216e3 * t75 * xTarget - 0.216e3 * t75 * x0 - 0.144e3 * t194 * t2 + 0.180e3 * t59 * stretchToTime * jmax + 0.36e2 * t166 * t245 + 0.36e2 * t123 * a0 + 0.144e3 * t99 * t16 - 0.36e2 * t193 * t20;
    double t257 = dc * x0;
    double t259 = t55 * amax * t43;
    double t280 = t37 * t43;
    double t283 = 0.72e2 * t25 + 0.72e2 * t25 * da + 0.36e2 * stretchToTime * dc * t211 + 0.72e2 * t61 * t245 - 0.72e2 * t47 * da + 0.72e2 * t1 * t72 - 0.180e3 * t24 - 0.144e3 * t187 * t184 - 0.72e2 * t47 + 0.36e2 * t8 * t51 + 0.36e2 * t153 * t280;
    double t284 = t32 * da;
    double t303 = -0.48e2 * t284 - 0.72e2 * t28 * jmax + 0.36e2 * t153 * t120 - 0.72e2 * t112 * t96 - 0.72e2 * t153 * t59 - 0.72e2 * t65 * t231 - 0.180e3 * t101 * a0 - 0.72e2 * t123 - 0.48e2 * t32 - 0.144e3 * t10 - 0.72e2 * t41 * t65;
    double t306 = xTarget * xTarget;
    double t316 = dc * t3;
    double t317 = t43 * stretchToTime;
    double t321 = dc * t15;
    double t330 = t2 * t16;
    double t335 = t2 * t9;
    double t352 = v0 * stretchToTime;
    double t355 = -0.108e3 * t160 * t59 * t43 + 0.72e2 * t316 * t16 * t317 - 0.30e2 * t321 * t184 + 0.144e3 * t316 * xTarget * v0 - 0.144e3 * t316 * x0 * v0 - 0.72e2 * t330 * t175 + 0.72e2 * t330 * t178 + 0.144e3 * t335 * t178 - 0.144e3 * t335 * t175 - 0.72e2 * t160 * amax * t20 + 0.144e3 * t68 * t19 + 0.72e2 * t75 * t194 - 0.72e2 * t42 * t43 * v0 - 0.72e2 * t190 * t96 + 0.216e3 * t330 * t352;
    double t361 = t32 * v0;
    double t376 = t9 * stretchToTime * t16;
    double t382 = t1 * x0;
    double t388 = t1 * xTarget;
    double t397 = 0.72e2 * t112 * t9 * v0 - 0.144e3 * t190 * t89 + 0.24e2 * t65 * t361 - 0.144e3 * t29 * t19 - 0.72e2 * t75 * da * t20 + 0.36e2 * t65 * t87 * stretchToTime - 0.24e2 * t33 * t19 - 0.144e3 * t153 * jmax * t376 + 0.72e2 * t112 * t157 - 0.6e1 * t129 + 0.288e3 * t160 * t382 - 0.72e2 * t153 * t37 * t20 - 0.288e3 * t202 * t388 + 0.144e3 * t202 * t41 * t19 + 0.216e3 * t193 * t352 * t16;
    double t406 = t68 * v0;
    double t410 = t9 * amax * t43;
    double t427 = t3 * da;
    double t440 = -0.144e3 * t190 * t93 * stretchToTime + 0.72e2 * t25 * t16 - 0.72e2 * t47 * t16 - 0.144e3 * t153 * t406 + 0.72e2 * t160 * t410 - 0.108e3 * t153 * t2 * t59 * t43 - 0.72e2 * t190 * v0 + 0.36e2 * t215 * a0 - 0.48e2 * t284 * t16 + 0.144e3 * t335 * x0 - 0.30e2 * t321 * amax - 0.72e2 * t427 * t28 * t43 + 0.36e2 * t153 * jmax * t87 * stretchToTime + 0.24e2 * t65 * t361 * da + 0.36e2 * t87 * a0;
    double t450 = t3 * t16 * t317;
    double t476 = 0.72e2 * t330 * t93 * t43 - 0.6e1 * t129 * da - 0.72e2 * t76 - 0.144e3 * t335 * xTarget + 0.72e2 * t153 * t450 - 0.288e3 * t160 * t388 + 0.144e3 * t427 * t19 * x0 + 0.144e3 * t160 * a0 * t203 - 0.48e2 * t32 * t16 - 0.144e3 * t65 * t376 - 0.24e2 * t33 * t245 + 0.144e3 * t153 * t84 - 0.144e3 * t427 * t19 * xTarget + 0.288e3 * t202 * t382 + 0.72e2 * t202 * t410;
    double t481 = -0.72e2 * dc * t20 * v0 * t3 + 0.36e2 * t9 * t20 * t2 + 0.48e2 * t32 * xTarget * t2 - 0.48e2 * t32 * x0 * t2 - 0.72e2 * t16 * t20 * t2 - 0.18e2 * t87 * t9 - 0.72e2 * t257 * t259 - 0.72e2 * t306 * t55 + 0.108e3 * t123 * a0 * t43 * t2 + t15 * t9;
    double t494 = t4 * t16;
    double t500 = t43 * t43;
    double t516 = 0.144e3 * t1 * t4 * xTarget - 0.24e2 * t118 * t280 - 0.72e2 * t82 * t450 + 0.144e3 * t82 * t406 - 0.144e3 * t207 * t83 * t43 + 0.72e2 * t36 * t494 + 0.36e2 * t59 * t317 * t3 - 0.18e2 * t16 * t500 * t55 - 0.18e2 * t43 * t87 * t2 - 0.216e3 * t41 * t11 * t16 + 0.72e2 * t36 * t259 - 0.144e3 * t257 * t38 - 0.72e2 * t257 * t494;
    
    coeffs[4] = 0.36e2 * t8 * t93 + 0.36e2 * t65 * t96 + 0.18e2 * t99 + 0.36e2 * t101 + 0.18e2 * t9 + 0.36e2 * t8 * a0 + 0.36e2 * t16 - 0.36e2 * t19 * jmax + 0.36e2 * t109 * dc - 0.36e2 * t112 * t113;
    coeffs[3] = t283 + t303;
    coeffs[2] = t210 + t254;
    coeffs[1] = t355 + t397 + t440 + t476;
    coeffs[0] = t58 + t133 + t481 + t516;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileWT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double frac;
    {
        double t1 = jmax * amax;
        double t6 = jmax * jmax;
        double t11 = 0.6e1 * jmax * root * a0;
        double t15 = pow(root, 0.2e1);
        double t17 = 0.3e1 * jmax * t15;
        double t18 = a0 * a0;
        frac = 0.1e1 / (-0.6e1 * t1 * a0 + 0.6e1 * t1 * root + 0.6e1 * t6 * v0 + (t11 - 0.6e1 * t6 * amax * stretchToTime - t17 - 0.3e1 * jmax * t18) * dc + 0.6e1 * t1 * da * root + (-t17 + t11) * dc * da);
    }
    t[1] = root / dc / jmax;
    t[2] = 0;
    {
        double t1 = a0 * a0;
        double t3 = jmax * jmax;
        double t6 = amax * amax;
        double t15 = root * t6;
        double t19 = stretchToTime * stretchToTime;
        t[3] = (t1 * a0 + 0.6e1 * xTarget * t3 + 0.3e1 * t6 * a0 - 0.6e1 * x0 * t3 + 0.6e1 * amax * root * jmax * stretchToTime - 0.3e1 * t15 + (0.3e1 * amax * t1 - 0.3e1 * amax * t19 * t3 - 0.6e1 * root * jmax * v0 + 0.3e1 * stretchToTime * t6 * jmax) * dc + (-0.3e1 * t1 * root - 0.3e1 * t15) * da - 0.6e1 * dc * amax * a0 * da * root) * frac;
    }
    t[4] = 0;
    {
        double t2 = stretchToTime * jmax;
        double t5 = a0 * a0;
        double t8 = jmax * jmax;
        double t16 = pow(root, 0.2e1);
        double t18 = 0.9e1 * a0 * t16;
        double t19 = amax * amax;
        double t23 = 0.6e1 * t5 * root;
        double t24 = amax * root;
        double t26 = 0.6e1 * t24 * t2;
        double t28 = 0.3e1 * t16 * root;
        double t30 = 0.3e1 * root * t19;
        double t33 = 0.6e1 * root * jmax * v0;
        double t34 = stretchToTime * stretchToTime;
        double t47 = 0.9e1 * amax * t16;
        double t49 = 0.12e2 * t24 * a0;
        double t57 = -0.6e1 * amax * a0 * t2 - 0.2e1 * t5 * a0 + 0.6e1 * xTarget * t8 + 0.6e1 * amax * v0 * jmax - 0.6e1 * x0 * t8 - t18 - 0.3e1 * t19 * a0 + t23 + t26 + t28 + t30 + (-t33 - 0.3e1 * amax * t34 * t8 + 0.6e1 * a0 * v0 * jmax - 0.6e1 * amax * t5 - 0.3e1 * stretchToTime * t19 * jmax - t47 + t49) * dc + (t23 + t30 + t28 + t26 - t18) * da + (-t33 - t47 + t49) * dc * da;
        t[5] = t57 * frac;
    }
    {
        double t1 = a0 * a0;
        double t3 = jmax * jmax;
        double t14 = pow(root, 0.2e1);
        double t16 = 0.3e1 * a0 * t14;
        double t17 = amax * amax;
        double t21 = 0.3e1 * t1 * root;
        double t23 = 0.6e1 * root * t17;
        double t24 = stretchToTime * jmax;
        double t29 = 0.6e1 * root * jmax * v0;
        double t32 = 0.6e1 * t24 * root * a0;
        double t42 = 0.3e1 * t24 * t14;
        double t44 = 0.6e1 * amax * t14;
        double t47 = 0.12e2 * amax * root * a0;
        double t55 = t1 * a0 - 0.12e2 * xTarget * t3 - 0.12e2 * amax * v0 * jmax + 0.12e2 * x0 * t3 + 0.6e1 * stretchToTime * t3 * v0 + t16 + 0.6e1 * t17 * a0 - t21 - t23 + (-0.3e1 * t24 * t1 + t29 + t32 - 0.6e1 * a0 * v0 * jmax + 0.6e1 * amax * t1 + 0.6e1 * stretchToTime * t17 * jmax - t42 + t44 - t47) * dc + (-t21 - t23 + t16) * da + (-t42 + t32 + t29 + t44 - t47) * dc * da;
        t[6] = t55 * frac;
    }
    t[7] = amax / jmax;    
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileTcT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    coeffs[1] = 1;
    coeffs[0] = 0;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileTcT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double frac;
    {
        double t1 = amax * amax;
        double t4 = 0.12e2 * t1 * amax * jmax;
        double t6 = jmax * jmax;
        double t18 = a0 * a0;
        frac = 0.1e1 / (t4 - 0.24e2 * stretchToTime * t1 * t6 + 0.24e2 * jmax * t1 * a0 * da * dc + 0.24e2 * amax * t6 * v0 * dc + (0.12e2 * t18 * amax * jmax + t4) * da);
    }
    t[1] = (dc * amax + a0) / da / dc / jmax;
    {
        double t1 = a0 * a0;
        double t2 = t1 * t1;
        double t4 = amax * amax;
        double t7 = t4 * amax;
        double t9 = stretchToTime * t7 * jmax;
        double t11 = t4 * t4;
        double t13 = jmax * jmax;
        double t14 = v0 * v0;
        double t19 = t7 * a0;
        t[2] = (0.3e1 * t2 - 0.6e1 * t4 * t1 + 0.12e2 * t9 - 0.12e2 * t11 + 0.12e2 * t13 * t14 + (0.12e2 * jmax * v0 * t1 - 0.12e2 * t19 - 0.12e2 * t4 * jmax * v0) * da * dc + (0.4e1 * t1 * a0 * amax - 0.12e2 * t19 + 0.24e2 * xTarget * t13 * amax - 0.24e2 * x0 * t13 * amax - 0.24e2 * t13 * v0 * stretchToTime * amax) * dc + (-0.12e2 * t1 * stretchToTime * amax * jmax + 0.12e2 * t9 - 0.12e2 * t11) * da) * frac;
    }
    t[3] = amax / jmax;
    {
        double t1 = a0 * a0;
        double t2 = amax * amax;
        t[4] = (-t1 - t2 - 0.2e1 * jmax * v0 * da * dc - 0.2e1 * dc * amax * a0 + (-0.3e1 * t2 + 0.2e1 * stretchToTime * amax * jmax) * da) / jmax / amax / da / 0.2e1;
    }
    t[5] = amax / jmax;
    {
        double t1 = a0 * a0;
        double t2 = t1 * t1;
        double t4 = amax * amax;
        double t5 = t4 * t1;
        double t7 = t4 * t4;
        double t8 = 0.12e2 * t7;
        double t9 = t4 * amax;
        double t13 = jmax * jmax;
        double t14 = v0 * v0;
        double t17 = jmax * v0;
        double t23 = t4 * jmax * v0;
        t[6] = (0.3e1 * t2 + 0.6e1 * t5 - t8 + 0.24e2 * stretchToTime * t9 * jmax + 0.12e2 * t13 * t14 + (0.12e2 * t17 * t1 - 0.24e2 * t9 * a0 + 0.12e2 * t23) * da * dc + (0.8e1 * t1 * a0 * amax - 0.24e2 * xTarget * t13 * amax + 0.24e2 * x0 * t13 * amax - 0.24e2 * t23) * dc + (0.24e2 * t17 * a0 * amax - 0.12e2 * t5 - t8) * da) * frac;
    }
    t[7] = amax / jmax;
}

void Stp7Formulars::calcCoeffsStretchedDoubleDecProfileTT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double t7 = amax * amax;
    double t16 = a0 * a0;
    
    coeffs[2] = 0.2e1 * da;
    coeffs[1] = 0;
    coeffs[0] = 0.2e1 * v0 * jmax * da * dc + t7 * da - 0.2e1 * amax * da * jmax * stretchToTime + 0.2e1 * dc * amax * a0 + t16 + t7;
}

void Stp7Formulars::calcTimeIntervallsStretchedDoubleDecProfileTT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime) {
    double frac;
    {
        double t1 = jmax * jmax;
        double t8 = a0 * a0;
        double t11 = amax * amax;
        double t13 = 0.3e1 * jmax * t11;
        frac = 0.1e1 / (-0.6e1 * t1 * v0 - 0.6e1 * jmax * amax * a0 * da + (-0.3e1 * jmax * t8 - t13) * da * dc + (-t13 + 0.6e1 * t1 * amax * stretchToTime) * dc);
    }
    t[1] = (dc * amax + a0) / da / dc / jmax;
    {
        double t1 = a0 * a0;
        double t4 = jmax * jmax;
        double t7 = amax * amax;
        double t8 = t7 * a0;
        double t10 = amax * root;
        double t15 = 0.3e1 * root * t7;
        double t22 = v0 * jmax;
        double t31 = 0.3e1 * t7 * amax;
        double t33 = t7 * stretchToTime  * jmax;
        double t42 = stretchToTime * stretchToTime;
        t[2] = (0.2e1 * t1 * a0 - 0.6e1 * xTarget * t4 + 0.6e1 * t8 - 0.6e1 * t10 * jmax * stretchToTime + t15 + 0.6e1 * x0 * t4 + (-0.6e1 * amax * stretchToTime * jmax * a0 + 0.6e1 * t22 * amax + t15 + 0.3e1 * root * t1 + 0.3e1 * t8) * da + (t31 - 0.6e1 * t33 + 0.6e1 * t22 * a0 + 0.6e1 * t10 * a0) * dc * da + (t31 + 0.3e1 * amax * t42 * t4 - 0.3e1 * t33 + 0.6e1 * amax * t1 + 0.6e1 * root * jmax * v0) * dc) * frac;
    }
    t[3] = root / dc / jmax;
    t[4] = 0;
    t[5] = t[3];
    {
        double t1 = jmax * jmax;
        double t9 = a0 * a0;
        double t11 = amax * amax;
        double t12 = t11 * a0;
        double t15 = 0.3e1 * root * t11;
        double t16 = amax * root;
        double t17 = jmax * stretchToTime;
        double t33 = 0.3e1 * t11 * amax;
        double t35 = t11 * stretchToTime * jmax;
        double t40 = 0.3e1 * amax * t9;
        double t50 = stretchToTime * stretchToTime;
        double t56 = -0.6e1 * x0 * t1 + 0.6e1 * xTarget * t1 + 0.6e1 * v0 * jmax * amax + t9 * a0 + 0.3e1 * t12 + t15 - 0.6e1 * t16 * t17 - 0.6e1 * stretchToTime * t1 * v0 + (t15 + 0.6e1 * t12 - 0.6e1 * amax * stretchToTime * jmax * a0 + 0.3e1 * root * t9) * da + (t33 - 0.3e1 * t35 - 0.3e1 * t17 * t9 + t40 + 0.6e1 * t16 * a0) * dc * da + (0.6e1 * root * jmax * v0 - 0.6e1 * t35 + t33 + t40 + 0.3e1 * amax * t50 * t1) * dc;
        t[6] = t56 * frac;
    }
    t[7] = amax/jmax;
}

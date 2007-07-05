// 
// File:   stp7Formulars.h
// Author: erik
//
// Created on 10. Juni 2007, 15:17
//

#ifndef _STP7FORMULARS_H
#define	_STP7FORMULARS_H

using namespace std;

#include <iostream>
#include <string>

class Stp7Formulars {
public:
    static void solveProfile(double t[8],
            string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double stretchToTime=0);
    
    static bool areValidTimeIntervalls(double t[8]);
private:
    static void calcCoeffs(double coeffs[7],
        string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double stretchToTime,
            double t[8]);
    
    static void calcTimeIntervalls(double t[8],
        string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
        double root, double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double stretchToTime);
    
    
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsProfileTT(double coeffs[7],
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    static void calcTimeIntervallsProfileTT(double t[8], double root,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsProfileWW(double coeffs[7],
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    static void calcTimeIntervallsProfileWW(double t[8], double root,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsProfileWT(double coeffs[7],
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    static void calcTimeIntervallsProfileWT(double t[8], double root,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsProfileTW(double coeffs[7],
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    static void calcTimeIntervallsProfileTW(double t[8], double root,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsDoubleDecProfile_W(double coeffs[7],
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc,
            double t[8]);
    static void calcTimeIntervallsDoubleDecProfile_W(double t[8], double root,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsDoubleDecProfile_T(double coeffs[7],
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc,
            double t[8]);
    static void calcTimeIntervallsDoubleDecProfile_T(double t[8], double root,
            double x0, double xTarget, double v0, double vmax,
            double a0, double amax, double jmax, double da, double dc);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedProfileTT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedProfileTT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedProfileWW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedProfileWW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedProfileTW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedProfileTW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedProfileWT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedProfileWT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileWW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileWW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileWcW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileWcW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileTcW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileTcW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
};


#endif	/* _STP7FORMULARS_H */


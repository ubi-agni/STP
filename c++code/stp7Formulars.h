/**
 * \file stp7Formulars.h
 * \author Erik Weitnauer
 */

#ifndef _STP7FORMULARS_H
#define	_STP7FORMULARS_H

using namespace std;

#include <iostream>
#include <string>
#include <stdexcept>

/**
 * Class solving the equations for 3rd order trajectories.
 * \author Erik Weitnauer
 * \date 2007
 *
 * This class is used for getting the correct values for the time inveralls of
 * a 3rd order trajectory. It must be provided with sufficient information about
 * the type of profile:
 *      - TT / TW / WT / WW profile
 *      - with / without cruising phase
 *      - double deceleration?
 *      - start with deacceleration since a0 > amax?
 *      - direction of cruising phase
 *      - initial, target and limit values for position, velocity, acceleration, jerk and duration
 *  
 * These different conditions lead to 18 possible different profile types.
 * Planning the trajectory for any of them is done in two steps:
 * 
 * 1) Calculation of the coeffecients.
 * The coefficient of the polynomial which roots are used to calculate the
 * solutions for the time values later.
 *
 * 2) For each root found do:
 * Calculate the time intervalls associated with this root and check whether they
 * are a valid solution (e.g. no negativ time intervalls).
 * 
 * For the case that no valid solution for the time values could be found - either due
 * to incorrectly passed information about the type of the profile or because of possible
 * numerical instabilities in the algorithm, a logic_error is thrown.
 *
 * For the equations for the different profile types
 * \see fastest.mw, stretch.mw and stretch_doubledec.mw maple worksheet files in "./maple/"
 * \see "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories";
 * R. Haschke, E. Weitnauer, H. Ritter; 2007
 *
 * \warning The solutions for the times are given back as time intervalls instead
 * of absolute tmie points. The index of the time array ranges from 1..7, so a
 * double[8] array must be passed.
 */
class Stp7Formulars {
public:
    /**
     * Most general method to calculate the solution for the time
     * intervalls, taking into account all other parameters passed.
	 * @throws logic_error if no solution could be found.
     */
    static void solveProfile(double t[8],
            string type, bool bCruise, bool bDoubleDec, bool bDecAcc, bool bMoveForward,
            double x0, double xTarget, double v0, double vmax,
			double a0, double amax, double jmax, double stretchToTime=0) throw(logic_error);
    
    /// method returns true, if all values t[1..7] are positive or zero.
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
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileTW(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileTW(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileWcT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileWcT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileWT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileWT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileTcT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileTcT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    ////////////////////////////////////////////////////////////////////////////
    static void calcCoeffsStretchedDoubleDecProfileTT(double coeffs[7],
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
    static void calcTimeIntervallsStretchedDoubleDecProfileTT(double t[8], double root,
        double x0, double xTarget, double v0, double vmax,
        double a0, double amax, double jmax, double da, double dc,
        double stretchToTime);
};


#endif	/* _STP7FORMULARS_H */


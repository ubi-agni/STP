// 
// File:   stp7.h
// Author: erik
// Smooth Trajectory Planner, 3rd order (Stp7)
//
// Created on 25. April 2007, 08:40
//

#ifndef _stp7_H
#define	_stp7_H

using namespace std;

#include <iostream>
#include <string>

class Stp7 {
public:
    // string constants for different profile types
    const static string PROFILE_TT;
    const static string PROFILE_WT;
    const static string PROFILE_TW;
    const static string PROFILE_WW;
    const static string PROFILE_STOP;
    
    // constructor
    Stp7(): _plannedProfile(false) {};
    
    // functions for getting information about the calculated profile
    bool isDoubleDecProfile() const;
    bool hasCruisingPhase() const;
    string getProfileType() const;
    string getDetailedProfileType() const;
    // 1 <= i <= 7. Returns the time of switch between phase (i) and (i+1) of
    // the profile.
    double getSwitchTime(int i) const;
    // 1 <= i <= 7. Returns the time length of phase (i).
    double getTimeIntervall(int i) const;
    double getDuration() const { return getSwitchTime(7); }
    // gives back in which time intervall the passed time lies inside. For
    // t = 0 return 1, for t >= duration returns 7.
    int getPhaseIndex(double t) const;
    
    // getters - they return a copy of the arrays.
    void getJerkArray(double j[8]) const;
    void getTimeArray(double t[8]) const;
    void getTimeIntArray(double t[8]) const;
    
    // function for getting the pos/vel/acc/jerk at different times >= 0
    void move(double t, double &x, double &v, double &a, double &j) const;
    double pos(double t) const;
    double vel(double t) const;
    double acc(double t) const;
    double jer(double t) const;
    
    // Function for calculating the time optimal profile. Returns the duration.
    double planFastestProfile(double x0, double xtarget, double v0, double vmax,
                              double a0, double amax, double jmax);
    
    // Function for scaling the profile to the given duration. Gives back the
    // new duration.
    double scaleToDuration(double newDuration);
    
    string toString() const;
    
    // The function returns an empty string if everything is correct, otherwise
    // an error description is returned.
    string testProfile() const;
    
    static void calcjTrack(double dt, double x0, double v0, double a0, double j,
                        double &newx, double &newv, double &newa);
    // Calculates a given 3rd order profile and writes the resulting position,
    // velocity and acceleration in the passed variables &x, &v, &a.
    // The t[] array must contain the time points (not intervalls) and indicies
    // go from t[1] to t[length].
    static void calcjTracks(double t[], double j[], int length, double x0,
                        double v0, double a0, double &x, double &v, double &a);
    
    // same as calcjTracks but with time intervalls instead of time points.
    static void calcjTracksTimeInt(double t[], double j[], int length,
              double x0, double v0, double a0, double &x, double &v, double &a);
    
        void sett(int i, double v);
protected:

private:
    // ?[0] is start condition, ?[7] is end condition.
    double _x[8], _v[8], _a[8], _t[8];  // t[i] is point in time, not intervall
    double _j[8];           // j[i] is jerk between x[i-1] and x[i]
    double _vmax, _amax, _jmax;
    string _sProfileType;
    bool _bIsddec;
    bool _bHasCruise;
    bool _plannedProfile;
    
    void planProfile();
    void planProfileNoCruise(int dir);
    
    void convertTimePointsToIntervalls();
    void convertTimeIntervallsToPoints();
    
    static bool stillOvershootsTimeInt(double t[8], double j[8], int dir,
                               double x0, double xTarget, double v0, double a0);
    static string findProfileTimeInt(double t[8], double j[8], int dir, double x0,
                              double xTarget, double v0, double a0, double amax,
                              double jmax);
    static void calc7st_opt_shiftTimeInt(double t[8], double j[8], int dir, double amax,
                                  double jmax, double v2, double a2,
                                  double &tDelta, double &t5, double &t7);
    static string getProfileString(double t[8]);
    
    static void adaptProfile(double t[8], double j[8], double xtarget, double a0, double v0, double x0);
    
    static bool stillTooShort(double t[8], double newDuration);
    
    static void shiftDoubleDecArea(double t[8], double j[8], double newDuration, 
                            double x0, double xTarget, double v0, double vmax,
                            double a0, double amax, double jmax);
    
    static void addAreaTimeInt(double deltaV, double amax, double jmax, double t[4]);
    
    static void removeAreaTimeInt(double t[4], double deltaV, double amax, double jmax);
    
    static void splitNoCruiseProfileTimeInt(double t[8], double j[8], double a0);
    
    void findProfileTypeStretchCanonical(double newDuration);
    void planProfileStretchDoubleDec(double newDuration, double dir, double da);
};

#endif	/* _stp7_H */


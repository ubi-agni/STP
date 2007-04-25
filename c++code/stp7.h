// 
// File:   stp7.h
// Author: erik
//
// Created on 25. April 2007, 08:40
//

#ifndef _stp7_H
#define	_stp7_H

using namespace std;

#include <iostream>
#include <string>
#include <exception>

class stpException: public exception
{
  virtual const char* what() const throw()
  {
    return "Logic error. Consider to call planFastestProfile(...) first.";
  }
};

//class stpException : public std::exception {
//public:
//    stpException(string sReason) throw() { What = sReason;};
//    ~stpException() throw() {};
//    const char* what() const throw() {return What.c_str;};
//private:
//    string What;
//};

class Stp7 {
public:
    // string constants for different profile types
    static const string PROFILE_TT;
    static const string PROFILE_WT;
    static const string PROFILE_TW;
    static const string PROFILE_WW;
    static const string PROFILE_STOP;
    
    // constructor
    Stp7(): _plannedProfile(false) {};
    
    // functions for getting information about the calculated profile
    bool isDoubleDecProfile() const;
    bool hasCruisingPhase() const;
    string getProfileType() const;
    // 1 <= i <= 7. Returns the time of switch between phase (i) and (i+1) of
    // the profile.
    double getSwitchTime(int i) const;
    // 1 <= 1 <= 7. Returns the time length of phase (i).
    double getTimeIntervall(int i) const;
    double getDuration() const { return getSwitchTime(7); }
    // gives back in which time intervall the passed time lies inside. For
    // t = 0 return 1, for t >= duration returns 7.
    int getPhaseIndex(double t) const;
    
    // getters - they return a copy of the arrays.
    double* getJerkArray() const;
    double* getTimeArray() const;
    
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
    
    void calcjTrack(double dt, double x0, double v0, double a0, double j,
                        double &newx, double &newv, double &newa) const;
    
    void planProfile();
};

// initialize Stp7 string constants...
const string Stp7::PROFILE_STOP = "stop profile";
const string Stp7::PROFILE_TT = "TT profile";
const string Stp7::PROFILE_TW = "TW profile";
const string Stp7::PROFILE_WT = "WT profile";
const string Stp7::PROFILE_WW = "WW profile";

#endif	/* _stp7_H */


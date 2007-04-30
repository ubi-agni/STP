/*****************************************************************************
 *  Smooth Trajectory Planner, 2nd order (Stp3)
 *  \author Robert Haschke, Erik Weitnauer
 *	\par Release
 *		$Id: STP.hh,v 1.4 2006/12/13 09:25:03 rhaschke Exp $
 *		$Name:  $ 
 ****************************************************************************/

#ifndef _stp3_H
#define	_stp3_H

using namespace std;

#include <iostream>
#include <string>

class Stp3 {
public:
    const static string PROFILE_T;
    const static string PROFILE_W;
    const static string PROFILE_STOP;
    
    // constructor
    Stp3(): _plannedProfile(false) {};
    
    // functions for getting information about the calculated profile
    string getProfileType() const;
    bool isDoubleDecProfile() const;
    bool isTrapezoid() const;
    // 1 <= i <= 3. Returns the time of switch between phase (i) and (i+1) of
    // the profile.
    double getSwitchTime(int i) const;
    // 1 <= i <= 3. Returns the time length of phase (i).
    double getTimeIntervall(int i) const;
    double getDuration() const { return getSwitchTime(3); }
    // gives back in which time intervall the passed time lies inside. For
    // t = 0 return 1, for t >= duration returns 3.
    int getPhaseIndex(double t) const;
    
    // getters - they return a copy of the arrays.
    void getAccArray(double* a) const;
    void getTimeArray(double* t) const;
    
    // function for getting the pos/vel/acc at different times >= 0
    void move(double t, double &x, double &v, double &a) const;
    double pos(double t) const;
    double vel(double t) const;
    double acc(double t) const;
    
    // Function for calculating the time optimal profile. Returns the duration.
    double planFastestProfile(double x0, double xtarget, double v0, double vmax,
                              double amax);
    
    // Function for scaling the profile to the given duration. Gives back the
    // new duration.
    double scaleToDuration(double newDuration);
    
    string toString() const;
protected:
    
private:
    // ?[0] is start condition, ?[3] is end condition.
    double _x[4], _v[4], _t[4];  // t[i] is point in time, not intervall
    double _a[4];           // a[i] is acceleration between x[i-1] and x[i]
    double _vmax, _amax;    // limit for acceleration and velocity
    string _sProfileType;   
    bool _plannedProfile;   // flag indication whether a profile was computed
    bool _bIsddec;          // flag indicating deceleration in first phase

    
    void calcaTrack(double dt, double x0, double v0, double a,
                        double &newx, double &newv) const;
    
    void planProfile();
};

std::ostream& operator<<(std::ostream& os, const Stp3& c);

#endif	/* _stp3_H */


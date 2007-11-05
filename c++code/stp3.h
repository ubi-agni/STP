/**
 * \file stp3.h
 * \author Robert Haschke, Erik Weitnauer
 */
#ifndef _stp3_H
#define	_stp3_H

using namespace std;

#include <iostream>
#include <string>

/**
 * Smooth Trajectory Planner, 2nd order (Stp3)
 * \author Robert Haschke, Erik Weitnauer
 * \date 2006, 2007
 *
 * This class is for planning time optimal 2nd order trajectories.
 * Given a start and target position, an inital and maximum value for velocity
 * and a maximum value for acceleration you pass these values to the 
 * planFastestProfile method. Afterward the resulting profile can be accessed by
 * various methods, e.g. getTimeArray and getAccArray.
 * To get the movement parameters at specific times, call the move(...)
 * method.
 *
 * \image html 3phases.jpg "typical time optimal 2nd order trajectory"
 * \image latex 3phases.eps "typical time optimal 2nd order trajectory"
 *
 * A time optimal profile can be stretched to any desired duration by calling the
 * scaleToDuration method. This can be used to synchornise several
 * movements made at the same time.
 *
 * For further information about the theory of the used algorithm
 * \see "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories";
 * R. Haschke, E. Weitnauer, H. Ritter; 2007
 *
 * \warning Both the time and the acceleration arrays start at index 1 instead of
 * 0 and therefore the arrays are double[4].
 */
class Stp3 {
public:
    const static string PROFILE_T;  ///< trapezoid shaped profile
    const static string PROFILE_W;  ///< wedge shaped profile
    const static string PROFILE_STOP; ///< fullstop profile
    
    Stp3(): _plannedProfile(false) {}; ///< constructor
    
    // functions for getting information about the calculated profile
    /// returns one of the defined string constants describing the shape of the acceleration graph
    string getProfileType() const;
    /// returns true, when both acceleration in first and third phase have same direction
    bool isDoubleDecProfile() const;
    /// returns true, when profile is trapezoid (t(2) is not zero)
    bool isTrapezoid() const;
    /// returns time of switch between phase (i) and (i+1) of the profile.
    double getSwitchTime(int i) const;
    /// returns duration of phase (i).
    double getTimeIntervall(int i) const;
    /// returns total duration of the trajectory.
    double getDuration() const { return getSwitchTime(3); }
    /// returns index of time intervall the passed times lies in
    int getPhaseIndex(double t) const;
    
    void getAccArray(double a[4]) const; ///< @param[out] a array of acceleration values
    void getTimeArray(double t[4]) const; ///< @param[out] t array of time points
    void getTimeIntArray(double t[4]) const; ///< @param[out] t array of time intervalls
    
    /// get the position, velocity and acceleration at passed time >= 0
    void move(double t, double &x, double &v, double &a) const;
    double pos(double t) const; ///< get the position at passed time >= 0
    double vel(double t) const; ///< get the velocity at passed time >= 0
    double acc(double t) const; ///< get the acceleration at passed time >= 0
    
    /// Function for calculating the time optimal profile. Read the results with
    /// e.g. getTimeArray(..) or move(...).
    double planFastestProfile(double x0, double xtarget, double v0, double vmax,
                              double amax);
    
    /// scale a planned profile to a longer duration
    double scaleToDuration(double newDuration);
    
    /// convert to string
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


/**
 * \file stp7.h
 * \author Erik Weitnauer
 */
 
#ifndef _stp7_H
#define	_stp7_H

using namespace std;

#include <iostream>
#include <string>

 /**
 * Smooth Trajectory Planner, 3rd order (Stp7)
 * \author Erik Weitnauer
 * \date 2007
 *
 * This class is for planning time optimal 3rd order trajectories.
 * Given a start and target position, an inital and maximum value for velocity
 * and acceleration and a maximum value for jerk (change of acceleration) you pass
 * these values to the planFastestProfile method. Afterward the resulting profile
 * can be accessed by various methods, e.g. getTimeArray and getJerkArray.
 * To get the movement parameters at specific times, call the move(...)
 * method.
 *
 * \image html 3phases.jpg "typical time optimal 2nd order trajectory"
 * \image latex 3phases.eps "typical time optimal 2nd order trajectory"
 *
 * A time optimal profile can be stretched to any desired duration by calling the
 * scaleToDuration method. This can be used to synchornize several
 * movements made at the same time.
 * 
 * Both planFastestProfile and scaleToDuration throw a logic_error in case
 * they were unable to find a solution. In praxis could occour for some numerically
 * unstable cases, e.g. when extremely stretching a movement.
 * There should be some kind of fallback behaviour if this occours. For example the
 * much simpler and therefore more stable 2nd order Trajectory planner could be used.
 * 
 * However, the only numerically problematic cases where arising when trying to
 * stretch a very short movement to a long time - which in praxis makes no sence,
 * since the movement itself is so small that it is barely recognisable and therefore
 * must not be considered when synchronizing movements of several joints.
 * To avoid these numerical extreme cases, the stretch algorithm is slightly modified:
 * In case the algorithm does not lead to an solution and the movement should be
 * stretched to a time more than MAX_STRETCH_FACTOR times of the optimal time, the
 * original movement is returned unaltered. This accords to doing the movement at
 * once and just waiting after finshing it.
 * 
 * For further information about the theory of the used algorithm
 * \see "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories";
 * R. Haschke, E. Weitnauer, H. Ritter; 2007
 *
 * \warning Both the time and the jerk arrays start at index 1 instead of
 * 0 and therefore the arrays are double[8].
 */

class Stp7 {
public:
    // string constants for different profile types
    static const string PROFILE_TT; ///< trapezoid - trapezoid shaped acceleration profile
	static const string PROFILE_WT; ///< wedge- trapezoid shaped acceleration profile
	static const string PROFILE_TW; ///< trapezoid - wedge shaped acceleration profile
	static const string PROFILE_WW; ///< wedge - wedge shaped acceleration profile
	static const string PROFILE_STOP; ///< stop profile
	
	/**
	 * Errors when attempting to stretch a movement to a time more than this factor
	 * times the original duration are not thrown. Instead the profile is returned
	 * unchanged.
	 * Default value is 10.
	 */
	static const double MAX_STRETCH_FACTOR;
    
    // constructor
    Stp7(): _plannedProfile(false) {};
    
    // functions for getting information about the calculated profile
    bool isDoubleDecProfile() const;
    bool hasCruisingPhase() const;
    string getProfileType() const;
    string getDetailedProfileType() const;
    /// 1 <= i <= 7. Returns the time of switch between phase (i) and (i+1) of
    /// the profile.
    double getSwitchTime(int i) const;
    /// 1 <= i <= 7. Returns the time length of phase (i).
    double getTimeIntervall(int i) const;
    double getDuration() const { return getSwitchTime(7); }
    /// gives back in which time intervall the passed time lies inside. For
    /// t = 0 return 1, for t >= duration returns 7.
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
    
    /// Function for calculating the time optimal profile. Returns the duration.
	/// @throws logic_error if no solution could be found.
    double planFastestProfile(double x0, double xtarget, double v0, double vmax,
                              double a0, double amax, double jmax) throw(logic_error);
    
    /** Function for scaling the profile to the given duration. Gives back the
     * new duration.
	 * In case no solution could be found there are two possibilities:
	 * If the movement should be stretched to more than 10 time of the original
	 * duration, it is returned unchanged.
	 * Otherwise a logic_error is thrown.
	 * @throws logic_error
	 */
    double scaleToDuration(double newDuration) throw(logic_error);
    
    /// Returns at which time the cruising phase ends.
    double getEndOfCruisingTime();
    
    string toString() const;
    
    /** The function throws a logic_error exception, if the limits for jerk, acc or vel
	 * are broken, or if there are any incorrect time intervalls.
	 * If everything is correct, it just returns doing nothing
	.* @throws logic_error
	 */
    void testProfile() const throw(logic_error);
	
	/// As testProfile(), but also checking, if the target position was reached.
	/// @throws logic_error
	void testProfile(double xtarget) const throw(logic_error);
    
    static void calcjTrack(double dt, double x0, double v0, double a0, double j,
                        double &newx, double &newv, double &newa);
    /** Calculates a given 3rd order profile and writes the resulting position,
     * velocity and acceleration in the passed variables &x, &v, &a.
     * The t[] array must contain the time points (not intervalls) and indicies
     * go from t[1] to t[length].
	 */
    static void calcjTracks(double t[], double j[], int length, double x0,
                        double v0, double a0, double &x, double &v, double &a);
    
    /// same as calcjTracks but with time intervalls instead of time points.
    static void calcjTracksTimeInt(double t[], double j[], int length,
              double x0, double v0, double a0, double &x, double &v, double &a);
    
    /// Manuelly set a time value in the time array describing the movement.
    /// Don't use it unless you are feeling adventurous - its just intended for testing ;)
    void sett(int i, double t);
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
    
	void planProfile() throw(logic_error);
	void planProfileNoCruise(int dir) throw(logic_error);
    
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
	void planProfileStretchDoubleDec(double newDuration, double dir, double da) throw(logic_error);
};

#endif	/* _stp7_H */


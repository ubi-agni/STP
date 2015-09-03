/**
 * \file stp1.hh
 * \author Robert Haschke
 */
#pragma once
#include "stp.hh"
#include <iostream>

/**
 * Trajectory Planner, 1st order (Stp1)
 * \author Robert Haschke
 * \date 2009
 *
 * This class is for planning simple linear trajectories with constant velocity.
 * Given a start and target position, and a maximum value for velocity
 * you pass these values to the planFastestProfile method. 
 * To get the movement parameters at specific times, call the move(...) method.
 *
 * A profile can be stretched to any desired duration by calling the
 * scaleToDuration method. This method can be used to synchronise several
 * movements made at the same time.
 * 
 * Both planFastestProfile and scaleToDuration throw a logic_error in case
 * they were unable to find a solution. In praxis this should never occour, so
 * you better take it serious if it does and provide some kind of fallback - 
 * for example stopping the motion of the joint immediately.
 *
 * For further information about the theory of the used algorithm
 * \see "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories";
 * R. Haschke, E. Weitnauer, H. Ritter; 2007
 *
 */
class Stp1 : public StpBase {
	friend std::ostream& operator<<(std::ostream& os, const Stp1& c);

public:
   Stp1();
    
   virtual double getDuration() const { return _T; }
	virtual double getEndOfCruisingTime() const { return _T; }

   /// get the position, velocity and acceleration at passed time >= 0
   void move(double t, double &x, double &v) const;
   virtual double pos(double t) const; ///< get the position at passed time >= 0
   virtual double vel(double t) const; ///< get the velocity at passed time >= 0
    
   /// Function for calculating the profile. 
   double planFastestProfile(double x0, double xtarget, double vmax)
		throw(std::logic_error);
    
   /// scale a planned profile to a longer duration
   virtual double scaleToDuration(double newDuration)
		throw(std::logic_error);
	
private:
   double _x[2];  // initial and target position
   double _v, _T; // (signed) velocity and duration of motion
};


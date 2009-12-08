#pragma once
#include "stp1.hh"
#include "stp3.hh"
#include "stp7.hh"

#define MIN_VALUE_EQ_ZERO (1.e-8)

/**
 * Smooth Trajectory Planner (STP)
 * \author Robert Haschke, Erik Weitnauer
 * \date 2007
 *
 * This class is able to plan a smooth trajectory of a single joint.
 * You can choose, whether 2nd or 3rd order equations for the movement should
 * be used.
 * 2nd order:
 * Acceleration impulses are given, the speed changes linear. Values for
 * maximum acceleration and maximum and initial speed must be provided.
 * 3rd order:
 * Enormously more complex computation. Jerk impulses are given, so the
 * acceleration changes linear and the speed changes in a quadratic manner.
 * Values for max. jerk as well as max. and initial acceleration and speed
 * must be provided.
 *
 * After getting the time optimal trajectory for the given movement
 * parameters, the movement can be stretched to any time, that is longer or
 * equal the optimal time.
 * By means of this stretching the movement of this single joint can be
 * synchronized with other joints.
 *
 * This class acts mainly as a fasade for the underlying classes stp7 and stp3,
 * which handle the 3rd and 2nd order case. 
 *
 * Problem / error handling:
 * In case the Stp7 class throws a logic_error, because it could not find a
 * proper solution for the movement, or in case a runtime error occours this
 * class falls back to the 2nd order equations and uses the Stp3 class to
 * compute them.
 * In case the Stp3 class throws an exception, an immediate shutdown is
 * performed. To continue with any motion, restart() must be called.
 *
 * For further information about the theory of the used algorithm
 * \see "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories";
 * R. Haschke, E. Weitnauer, H. Ritter; 2007
 */

class STP {
	friend std::ostream& operator<<(std::ostream& os, const STP& c);

public:
	typedef unsigned char order;

   /// Constructor
	/// Complexity of used equations can be passed.
   STP(order o);

   // destructor
   ~STP();

   /**
    * Set limit values for movement parameters.
    * In 3rd order mode you have to provide values for vel, acc, jerk.
    * In 2nd order mode only for vel and acc.
	 * In 1st order mode only vel is needed.
    */
   void setMax (double maxvel, double maxacc, double maxjerk);
   void setMax (double maxvel, double maxacc) {setMax (maxvel, maxacc, jmax);}
	void setMax (double maxvel) {setMax (maxvel, amax, jmax);}

   /**
	 * Function for calculating the time optimal profile.
	 * In 2nd order mode you provide target, start pos. and cur. velocity.
	 * In 3rd order mode you additionally provide the cur. acceleration.
	 * Returns the duration.
	 */
   double planFastestProfile(double xtarget, double x0, double v0=0, double a0=0);
    
   /// Scales an already planned profile to a longer duration.
   /// Returns new duration.
   double scaleToDuration(double newDuration);

   /// Returns the duration the full movement will need.
   double getDuration() const {return pCurrent->getDuration();}

   /// Returns the time at which the cruising time ends.    
   double getEndOfCruisingTime() const
		{return pCurrent->getEndOfCruisingTime();}

   /// 1st oder: get the pos and velocity at passed time t >= 0
   void move(double t, double &x, double &v) const
      {double a,j; move (t, x, v, a, j);}

   /// 2nd oder: get the pos, velocity and acceleration at passed time t >= 0
   void move(double t, double &x, double &v, double &a) const
      {double j; move (t, x, v, a, j);}

   /// 3rd oder: get the pos, vel, acc and jerk at passed time >= 0
   void move(double t, double &x, double &v, double &a, double &j) const;

   double pos(double t) const {return pCurrent->pos(t);}
   double vel(double t) const {return pCurrent->vel(t);}
   double acc(double t) const {return pCurrent->acc(t);}
   double jerk(double t) const {return pCurrent->jerk(t);}

private:
   STP(const STP&);

private:
   Stp7* pStp7;
   Stp3* pStp3;
	Stp1* pStp1;
	StpBase* pCurrent;

	order desiredOrder, usedOrder;
   double vmax; ///< limit for velocity
   double amax; ///< limit for acceleration
   double jmax; ///< limit for jerk
	double xtarget; ///< target position
};

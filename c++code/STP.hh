#ifndef __STP_HH
#define __STP_HH

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
public:
   /// Constructor
	/// Complexity of used equations can be passed.
   STP(bool use3rdOrder=true);

   STP(const STP&);

   // destructor
   ~STP();

   /**
    * Set limit values for movement parameters.
    * In 3rd order mode you have to provide values for vel, acc, jerk.
    * In 2nd order mode only for vel and acc.
    */
   void setMax (double maxvel, double maxacc, double maxjerk);
   void setMax (double maxvel, double maxacc)
      {setMax (maxvel, maxacc, jmax);}

   /**
	 * Function for calculating the time optimal profile.
	 * In 2nd order mode you provide target, start pos. and cur. velocity.
	 * In 3rd order mode you additionally provide the cur. acceleration.
	 * Returns the duration.
	 */
   double planFastestProfile(double xtarget, double x0, double v0, double a0=0);
    
   /// Scales an already planned profile to a longer duration.
   /// Returns new duration.
   double scaleToDuration(double newDuration);

   /// Returns the duration the full movement will need.
   double getDuration();

   /// Returns the time at which the cruising time ends.    
   double getEndOfCruisingTime() const;

   /// 2nd oder: get the pos, velocity and acceleration at passed time >= 0
   void move(double t, double &x, double &v, double &a) const
      {double j; move (t, x, v, a, j);}

   /// 3rd oder: get the pos, vel, acc and jerk at passed time >= 0
   void move(double t, double &x, double &v, double &a, double &j) const;

   double pos(double t) const; ///< get the position at passed time >= 0
   double vel(double t) const; ///< get the velocity at passed time >= 0
   double acc(double t) const; ///< get the acceleration at passed time >= 0
   double jerk(double t) const; ///< get the jerk at passed time >= 0

   /// convert to string
   std::string toString() const;

private:
   Stp7* pStp7;
   Stp3* pStp3;
   bool  use3rdOrder, fallback2ndOrder, shutdown;
   double vmax; ///< limit for velocity
   double amax; ///< limit for acceleration
   double jmax; ///< limit for jerk
};

#endif // __STP_HH

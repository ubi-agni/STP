/**
 * Multi-Axes Smooth Trajectory Planner (MultiSTP)
 * \author Robert Haschke, Erik Weitnauer
 * \date 2007, 2008
 * 
 * With this class you can plan a smooth trajectory with a given number of
 * joints in a way all joints finish there movement at the same time. You
 * can choose, whether 2nd or 3rd order equations for the movement should
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
 * This class uses a STP object for every joint.
 *
 * For further information about the theory of the used algorithm
 * \see "On-Line Planning of Time-Optimal, Jerk-Limited Trajectories";
 * R. Haschke, E. Weitnauer, H. Ritter; 2007
 */
 
#pragma once
#include <vector>
#include "STP.hh"

class MultiSTP {
public:
   typedef std::vector<double> ValueArray;
   enum SyncMode { synced, fastest };
   enum RescheduleMode { onFirst = 0, onLast };

	/** Constructor
	 * Takes argument how many joints and which order of equations to use.
	 */  
   MultiSTP (int nAxes, STP::order o);
	~MultiSTP();

   /// Plans the fastest profile, using the start and goal conditions
   /// passed in double vectors (with one element per joint).
   double planFastestProfile (const ValueArray& vGoal, const ValueArray& vCurrent, 
                              const ValueArray& vVel, const ValueArray& vAcc);
   double planFastestProfile (const ValueArray& vGoal, const ValueArray& vCurrent, 
                              const ValueArray& vVel) 
      {return planFastestProfile (vGoal, vCurrent, vVel, vZero);}
   double planFastestProfile (const ValueArray& vGoal, const ValueArray& vCurrent) 
      {return planFastestProfile (vGoal, vCurrent, vZero, vZero);}
   
   /// Scales the whole movement to a new duration, which must be at least
   /// as big as the maximal optimal duration of the joints. 
   bool   scaleToDuration (double dNewDuration);
   
   /// Gives back the duration of movement of slowest joint.
   double getDuration () const {return dDuration;}

   /**
    * Sets the limits for velocity, acceleration and jerk for each joint.
    * The change will only take effect on the next planning. */
   template <typename T>
   void setMax (const std::vector<T>& vMaxVel, const std::vector<T>& vMaxAcc,
                const std::vector<T>& vMaxJer);
   // Set only new limits for velocity and acceleration, keep jerk limits
	template <typename T>
   void setMax (const std::vector<T>& vMaxVel, const std::vector<T>& vMaxAcc);
   
   /**
    * Sets the synchronization mode for the joints, which can either be synched
    * or fastest. In synched mode, all joints will finish their movement at the
    * same time, in fastest mode, all joints will finish their movement as fast
    * as possible.
    * The change will only take effect on the next planning. */
   void setSyncMode(SyncMode mode) {sync = mode;}
   
   /**
    * Sets the reschedule mode for the movement, which can either be onFirst
    * or onLast. In onFirst mode, all joints will switch to the next target as
    * soon as the first joint enters the decceleration phase after cruising. In
    * onLast mode, which is the default mode, the joint targets will switch to the
    * next target in the moment the last joint starts its deceleration.
    * The change will only take effect on the next planning. */
   void setRescheduleMode(RescheduleMode mode) { reschedule = mode;}
   
   /// Returns true if movement already is in the last deceleration process.
   bool isAfterCruising (double t) const {return t >= dSwitchTime;}
   
   /// Returns the number of joints.
   unsigned int getNumAxes () const {return nAxes;}

   /// Get the positions of all joints at a specific time.
   const ValueArray& pos (double t) const;
   /// Get the velocity of all joints at a specific time.
   const ValueArray& vel (double t) const;
   /// Get the acceleration of all joints at a specific time.
   const ValueArray& acc (double t) const;
   /// Get the jerk of all joints at a specific time.
   const ValueArray& jerk (double t) const;

   /// Get position vector computed by last call to pos(t)
   const ValueArray& getPos() const {return vPos;}
   /// Get velocity vector computed by last call to pos(t)
   const ValueArray& getVel() const {return vVel;}
   /// Get acceleration vector computed by last call to pos(t)
   const ValueArray& getAcc() const {return vAcc;}
   /// Get jerk vector computed by last call to pos(t)
   const ValueArray& getJerk() const {return vJer;}

protected:
   /* internal method to compute the time to switch to next target */
   void findSwitchTime ();

protected:
   typedef std::vector<STP*> STPArray;
   STPArray vSTP;
   mutable ValueArray vPos;
   mutable ValueArray vVel;
   mutable ValueArray vAcc;
   mutable ValueArray vJer;
   const   ValueArray vZero;

   SyncMode       sync;
   RescheduleMode reschedule;

   const unsigned int nAxes;
   double             dMinDuration;
   double             dDuration;
   double		       dSwitchTime;
};

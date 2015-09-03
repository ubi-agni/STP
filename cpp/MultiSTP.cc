/******************************************************************************
 *  Multi-Axes  Smooth Trajectory Planner (MultiSTP)
 *  \author Robert Haschke, Erik Weitnauer
 *	\par Release
 *		$Id: MultiSTP.cc,v 1.2 2006/11/06 10:11:43 rhaschke Exp $
 *		$Name:  $ 
 *****************************************************************************/

#include <Logger.hh>
#include "MultiSTP.hh"
#include <assert.h>

GETLOGGER ("stp");

using namespace std;

MultiSTP::MultiSTP (int axes, STP::order o) :
   vPos(axes), vVel (axes), vAcc (axes), vJer(axes), vZero(axes), nAxes (axes)
{
	for (int i=0; i<axes; i++) {
		vSTP.push_back(new STP(o));
	}
   dDuration = dMinDuration = 0.0;
   sync = synced;
   reschedule = onLast;
}

MultiSTP::~MultiSTP() {
	for (STPArray::iterator it = vSTP.begin(); it != vSTP.end(); ++it) {
		delete (*it);
	}
}

template <typename T>
void MultiSTP::setMax (const std::vector<T>& vMaxVel, 
                       const std::vector<T>& vMaxAcc,
                       const std::vector<T>& vMaxJer) {
   assert (vMaxJer.size () == nAxes && 
           vMaxVel.size () == nAxes && vMaxAcc.size () == nAxes);

   for (unsigned int i=0; i < nAxes; i++) {
      vSTP[i]->setMax (vMaxVel[i], vMaxAcc[i], vMaxJer[i]);
   }
}

template <typename T>
void MultiSTP::setMax (const std::vector<T>& vMaxVel, 
                       const std::vector<T>& vMaxAcc) {
   assert (vMaxVel.size () == nAxes && vMaxAcc.size () == nAxes);

   for (unsigned int i=0; i < nAxes; i++) {
      vSTP[i]->setMax (vMaxVel[i], vMaxAcc[i]);
   }
}

template void MultiSTP::setMax (const std::vector<float>& vMaxVel, 
                                const std::vector<float>& vMaxAcc,
                                const std::vector<float>& vMaxJer);
template void MultiSTP::setMax (const std::vector<double>& vMaxVel, 
                                const std::vector<double>& vMaxAcc,
                                const std::vector<double>& vMaxJer);
template void MultiSTP::setMax (const std::vector<float>& vMaxVel, 
                                const std::vector<float>& vMaxAcc);
template void MultiSTP::setMax (const std::vector<double>& vMaxVel, 
                                const std::vector<double>& vMaxAcc);

double MultiSTP::planFastestProfile (const ValueArray& vGoal, 
                                     const ValueArray& vCurrent,
                                     const ValueArray& vVel,
                                     const ValueArray& vAcc) {
   assert (vGoal.size () == nAxes && vCurrent.size () == nAxes &&
           vVel.size () == nAxes && vAcc.size() == nAxes);

   /* compute minimally possible duration == slowest joint */
   dMinDuration = 0;
   for (unsigned int i=0; i < nAxes; i++) {
      dMinDuration = std::max (dMinDuration, 
         vSTP[i]->planFastestProfile (vGoal[i], vCurrent[i], vVel[i], vAcc[i]));
      SDEBUG ("Joint " << i << " (fastest): " << *vSTP[i]);
   }

   dDuration = dMinDuration;
   if (sync == synced) scaleToDuration (dMinDuration);
 	else findSwitchTime(); // because scaleToDuration will call this by itself
 
   return dMinDuration;
}

bool MultiSTP::scaleToDuration (double dNewDuration) {
	if (dNewDuration < dMinDuration) return false; /* only enlarge duration */
	for (unsigned int i=0; i < nAxes; i++) {
		try {
			vSTP[i]->scaleToDuration (dNewDuration);
		} catch (const std::exception &e) {
			SWARN ("caught exception: " << e.what());
		} catch (...) {
			SWARN ("caught unknown exception");
		}
		SDEBUG("Joint " << i << " (stretched): " << *vSTP[i]);
	}
	dDuration = dNewDuration;
	findSwitchTime ();
	return true;
}

void MultiSTP::findSwitchTime () {
   dSwitchTime = vSTP[0]->getEndOfCruisingTime();

   if (reschedule == onFirst) {
      for (unsigned int i=1; i < nAxes; i++)
         dSwitchTime = std::min (dSwitchTime, 
                                 vSTP[i]->getEndOfCruisingTime());
   } else { // onLast
      for (unsigned int i=1; i < nAxes; i++)
         dSwitchTime = std::max (dSwitchTime, 
                                 vSTP[i]->getEndOfCruisingTime());
   }
}

const MultiSTP::ValueArray& MultiSTP::pos (double t) const {
	for (unsigned int i=0; i < nAxes; i++) vPos[i] = vSTP[i]->pos (t);
	return vPos;
}
const MultiSTP::ValueArray& MultiSTP::vel (double t) const {
   for (unsigned int i=0; i < nAxes; i++) vVel[i] = vSTP[i]->vel (t);
   return vVel;
}
const MultiSTP::ValueArray& MultiSTP::acc (double t) const {
   for (unsigned int i=0; i < nAxes; i++) vAcc[i] = vSTP[i]->acc (t);
   return vAcc;
}
const MultiSTP::ValueArray& MultiSTP::jerk (double t) const {
   for (unsigned int i=0; i < nAxes; i++) vJer[i] = vSTP[i]->jerk (t);
   return vJer;
}

/**
 * \file STP.cc
 * \author Erik Weitnauer
 * \date 2007
 */

#include "STP.hh"
#include "tools.hh"

using namespace std;

STP::STP (bool use3rdOrder) : pStp7(0), pStp3(0), use3rdOrder(use3rdOrder) {
	if (use3rdOrder) pStp7 = new Stp7();
	pStp3 = new Stp3();
	vmax = 1;
	amax = 1;
	jmax = 1;
	fallback2ndOrder = false;
	shutdown = false;
}

STP::~STP() {
 	if (use3rdOrder) delete pStp7;
 	delete pStp3;
}

STP::STP(const STP &other)
{
	STP (other.use3rdOrder);

	vmax = other.vmax;
	amax = other.amax;
	jmax = other.jmax;
}

void STP::setMax (double maxvel, double maxacc, double maxjerk) {
   // silently ignore too small values (and keep old values)???
   if (!isZero(maxvel))  vmax = fabs(maxvel); 
   if (!isZero(maxacc))  amax = fabs(maxacc);
   if (!isZero(maxjerk)) jmax = fabs(maxjerk);
}

double STP::planFastestProfile(double xtarget, double x0, double v0, double a0) {
	fallback2ndOrder = shutdown = false;

   if (use3rdOrder) {
		try {
         // in case an error occurs during profile stretching, 
         // we will need the 2nd order profile
         pStp3->planFastestProfile(x0, xtarget, v0, vmax, amax);
         // compute the 3rd order profile
			return pStp7->planFastestProfile(x0, xtarget, v0, vmax, a0, amax, jmax);
		} catch(exception &e) {
			cout << "Error calculating 3rd order profile: " << endl << e.what() << endl;
			cout << "Falling back to 2nd order planning..." << endl;
         fallback2ndOrder = true;
		}
   }

   // if we get here, either use3rdOrder==false or 3rd profile failed
   try {
      return pStp3->planFastestProfile(x0, xtarget, v0, vmax, amax);
   } catch(exception &e) {
      cout << "Error calculating 2nd order profile: " << endl << e.what() << endl;
      cout << "Immediate shutdown!!!" << endl;
      shutdown = true;
      return 0;
   }
}

double STP::scaleToDuration(double dNewDuration) {
	if (use3rdOrder && !fallback2ndOrder) {
		try {
			return pStp7->scaleToDuration(dNewDuration);
		} catch(exception &e) {
			cout << "Error stretching 3rd order profile: " << endl << e.what() << endl;
			cout << "Falling back to 2nd order planning..." << endl;
         fallback2ndOrder = true;
      }
   }

   try {
      return pStp3->scaleToDuration(dNewDuration);
   } catch(exception &e) {
      cout << "Error stretching 2nd order profile: " << endl << e.what() << endl;
      cout << "Immediate shutdown!!!" << endl;
      shutdown = true;
      return 0;
	}
}

double STP::getDuration() {
	if (shutdown) return 0;
	if (use3rdOrder && !fallback2ndOrder) return pStp7->getDuration();
	else return pStp3->getDuration();
}

double STP::getEndOfCruisingTime() const {
	if (shutdown) return 0;
	if (use3rdOrder && !fallback2ndOrder)
		return pStp7->getEndOfCruisingTime();
	else 
      return pStp3->getEndOfCruisingTime(); 
}

void STP::move(double t, double &x, double &v, double &a, double &j) const {
	if (shutdown) {x = 0; v = 0; a = 0; j = 0; return;}
	if (use3rdOrder && !fallback2ndOrder) pStp7->move(t, x, v, a, j);
	else pStp3->move(t, x, v, a);
}

double STP::pos(double t) const {
	if (shutdown) return 0;
	if (use3rdOrder && !fallback2ndOrder) return pStp7->pos(t);
	else return pStp3->pos(t);
}
double STP::vel(double t) const {
	if (shutdown) return 0;
	if (use3rdOrder && !fallback2ndOrder) return pStp7->vel(t);
	else return pStp3->vel(t);
}
double STP::acc(double t) const {
	if (shutdown) return 0;
	if (use3rdOrder && !fallback2ndOrder) return pStp7->acc(t);
   else return pStp3->acc(t);
}
double STP::jerk(double t) const {
	if (shutdown) return 0;
	if (use3rdOrder && !fallback2ndOrder) return pStp7->jer(t);
   else return 0;
}

string STP::toString() const {
	if (shutdown) return "Shutdown performed. Movement stopped.";
   if (use3rdOrder && !fallback2ndOrder) return pStp7->toString();
   else return pStp3->toString();
}

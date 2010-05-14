/**
 * \file STP.cc
 * \author Erik Weitnauer
 * \date 2007
 */

#include "STP.hh"
#include "tools.hh"

using namespace std;

STP::STP (order o) : pStp7(0), pStp3(0), pStp1(0), pCurrent(0), desiredOrder(o) {
	switch (o) {
	  // do not break to initialize fallback planners
	  case 3: pStp7 = new Stp7(); 
	  case 2: pStp3 = new Stp3();
	  case 1: pStp1 = new Stp1();
	}
	vmax = 1;
	amax = 1;
	jmax = 1;

	usedOrder = desiredOrder;
	this->xtarget = 0;
}

STP::~STP() {
	if (pStp7) delete pStp7;
	if (pStp3) delete pStp3;
	if (pStp1) delete pStp1;
}

void STP::setMax (double maxvel, double maxacc, double maxjerk) {
   // silently ignore too small values (and keep old values)???
   if (!isZero(maxvel))  vmax = fabs(maxvel); 
   if (!isZero(maxacc))  amax = fabs(maxacc);
   if (!isZero(maxjerk)) jmax = fabs(maxjerk);
}

double STP::planFastestProfile(double xtarget, double x0, double v0, double a0) {
	this->xtarget = xtarget;
	for (order o=desiredOrder; o >= 1; --o) {
		usedOrder = o;
		try {
			switch (o) {
			  case 3: 
				  // compute 3rd order profile
				  pCurrent = pStp7;
				  return pStp7->planFastestProfile(x0, xtarget, v0, vmax, a0, amax, jmax);
			  case 2:
				  // compute 2nd order profile
				  pCurrent = pStp3;
				  return pStp3->planFastestProfile(x0, xtarget, v0, vmax, amax);
			  case 1:
				  // compute 1st order profile
				  pCurrent = pStp1;
				  return pStp1->planFastestProfile(x0, xtarget, vmax);
			}
		} catch (const logic_error &e) {
			cout << "Error calculating profile of " << (int)o << ". order: " 
				  << e.what() << endl << *this << endl;
		}
	}
}


double STP::scaleToDuration(double dNewDuration) {
	double x0,v0,a0,j0;
	move(0, x0, v0, a0, j0);
	for (order o=usedOrder; o >= 1;) {
		usedOrder=o;
		try {
			switch (o) {
			  case 3: 
				  // compute 3rd order profile
				  pCurrent = pStp7;
				  return pStp7->scaleToDuration(dNewDuration);
			  case 2:
				  // compute 2nd order profile
				  pCurrent = pStp3;
				  return pStp3->scaleToDuration(dNewDuration);
			  case 1:
				  // compute 1st order profile
				  pCurrent = pStp1;
				  return pStp1->scaleToDuration(dNewDuration);
			}
		} catch (const logic_error &e) {
			cout << "Error stretching profile of " << (int)o << ". order to "
				  << dNewDuration << ": " << e.what() << endl << *this << endl;

			// plan initial profile of lower order
			order oldDesired = desiredOrder;
			desiredOrder = --o;
			planFastestProfile (xtarget, x0, v0, a0);
			desiredOrder = oldDesired;
		}
	}
	return 0; // this should not happen
}

void STP::move(double t, double &x, double &v, double &a, double &j) const {
	switch (usedOrder) {
	  case 3: pStp7->move(t, x, v, a, j); break;
	  case 2: pStp3->move(t, x, v, a); j=0; break;
	  case 1: pStp1->move(t, x, v); a=j=0; break;
	}
}

std::ostream& operator<<(std::ostream& os, const STP& c) { 
	switch (c.usedOrder) {
		case 3: os << *c.pStp7; break;
		case 2: os << *c.pStp3; break;
		case 1: os << *c.pStp1; break;
	}
	return os;
}

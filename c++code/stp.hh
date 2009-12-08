#pragma once
#include <stdexcept>
#include <string>

class StpBase {
public:
	StpBase () {}
	virtual ~StpBase () {};

   virtual double getDuration() const = 0;
	virtual double getEndOfCruisingTime() const = 0;

   virtual double pos(double t) const = 0;
   virtual double vel(double t) const = 0;
   virtual double acc(double t) const {return 0;}
   virtual double jerk(double t) const {return 0;}
    
   /// scale a planned profile to a longer duration
   virtual double scaleToDuration(double newDuration)
		throw(std::logic_error) = 0;
};

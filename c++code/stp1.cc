#include "stp1.hh"
#include "tools.hh"

Stp1::Stp1 () : _v(0), _T(0) {
	_x[0] = _x[1] = 0;
}

void Stp1::move(double t, double &x, double &v) const {
	v = _v;
	if (t >= _T) x = _x[1];
	else x = _x[0] + _v * t;
}

double Stp1::pos(double t) const {
	if (t >= _T) return _x[1];
	return _x[0] + _v * t;
}

double Stp1::vel(double t) const {
	return _v;
}

double Stp1::planFastestProfile(double x0, double xtarget, double vmax)
	throw(std::logic_error) 
{
	_x[0] = x0; _x[1] = xtarget;

	// check, whether vmax and amax are greater than zero
	if (isNegative(vmax)) throw std::invalid_argument("vmax must be positive");
	
	_v = xtarget < x0 ? -vmax : vmax;
	_T = (xtarget - x0) / _v;

	return _T;
}

double Stp1::scaleToDuration(double dNewDuration) throw(std::logic_error) {
	if (dNewDuration <= _T) return _T; // only enlarge duration
	_T = dNewDuration;
	_v = (_x[1] - _x[0]) / _T;
	return _T;
}

std::ostream& operator<<(std::ostream& os, const Stp1& c) { 
	os << "1st order trajectory: v=" << c._v << " T=" << c._T;
	return os;
}

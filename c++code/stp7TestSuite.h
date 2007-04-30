// 
// File:   stp7TestSuite.h
// Author: erik
//
// Created on 28. April 2007, 16:07
//

#ifndef _stp7TestSuite_H
#define	_stp7TestSuite_H

#include "stp7.h"
#include <cxxtest/TestSuite.h>

class Stp7TestSuite: public CxxTest::TestSuite {
public:
    void testBasics( void ) {
        Stp7 stp;
        TS_ASSERT_DIFFERS(stp.toString(), "");
        TS_ASSERT_THROWS_NOTHING(stp.planFastestProfile(0, 30, 0, 6, 0, 4, 2));
        TS_ASSERT_DIFFERS(stp.toString(), "");
        TS_ASSERT_DELTA(stp.getDuration(), 8.46,0.01);
        TS_ASSERT_DELTA(stp.pos(8.46),30,0.05);
        TS_ASSERT_DELTA(stp.vel(8.46),0,0.1);
        TS_ASSERT_DELTA(stp.acc(8.46),0,0.1);
    };
    
    void testCruiseProfilesStandard(void) {
        Stp7 stp;
        // TT profile
        stp.planFastestProfile(-5,5,0,3,0,2,2);
        TS_ASSERT_DELTA(stp.getDuration(),5.8333,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_TT);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(!stp.isDoubleDecProfile());
        
        // TW profile
        stp.planFastestProfile(-5,6,-1,3,1.5,2,1.25);
        TS_ASSERT_DELTA(stp.getDuration(),6.6834,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_TW);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(!stp.isDoubleDecProfile());
        
        // WT profile
        stp.planFastestProfile(-5,5,2,3,0,2,2);
        TS_ASSERT_DELTA(stp.getDuration(),4.8190,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_WT);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(!stp.isDoubleDecProfile());
        
        // WW profile
        stp.planFastestProfile(-4,4,0,2,0,2,1);
        TS_ASSERT_DELTA(stp.getDuration(),6.8284,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_WW);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(!stp.isDoubleDecProfile());
    }
    
    void testCruiseProfilesSpecial(void) {
        Stp7 stp;
        // a0>amax, double deceleration, TT
        stp.planFastestProfile(-3,3,-2,2,3,1.5,2);
        TS_ASSERT_DELTA(stp.getDuration(),6.0990,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_TT);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(stp.isDoubleDecProfile());
        
        // a(1.5)>amax, double deceleration, WW
        stp.planFastestProfile(-4,4,1.5,2,1.5,2,1.5);
        TS_ASSERT_DELTA(stp.getDuration(),5.1037,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_WW);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(stp.isDoubleDecProfile());
        
        // a0>amax, double deceleration, WW
        stp.planFastestProfile(-4,4,1,2,2.5,2,2);
        TS_ASSERT_DELTA(stp.getDuration(),4.8248,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_WW);
        TS_ASSERT(stp.hasCruisingPhase());
        TS_ASSERT(stp.isDoubleDecProfile());
    }
    
    void testNoCruiseProfilesStandard(void) {
        Stp7 stp;
        // WW profile
        stp.planFastestProfile(4,-4,0,3,0,2,1);
        TS_ASSERT_DELTA(stp.getDuration(),6.3496,0.0001);
        TS_ASSERT_EQUALS(stp.getProfileType(), Stp7::PROFILE_WW);
        TS_ASSERT(!stp.hasCruisingPhase());
        TS_ASSERT(!stp.isDoubleDecProfile());
    }
};

#endif	/* _stp7TestSuite_H */


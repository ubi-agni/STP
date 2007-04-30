// 
// File:   stp3TestSuite.h
// Author: erik
//
// Created on 28. April 2007, 13:00
//

#ifndef _stp3TestSuite_H
#define	_stp3TestSuite_H

#include <cxxtest/TestSuite.h>
#include "stp3.h"

class Stp3TestSuite : public CxxTest::TestSuite {
public:
    void setUp() {}
    
    void tearDown() {;}
    
    void testBasics( void ) {
        Stp3 stp3;
        TS_ASSERT_DIFFERS(stp3.toString(), "");
        TS_ASSERT_THROWS_NOTHING(stp3.planFastestProfile(0, 10, 0, 6, 4));
        TS_ASSERT_DIFFERS(stp3.toString(), "");
        TS_ASSERT_DELTA(stp3.getDuration(), 3.16,0.01);
        TS_ASSERT_DELTA(stp3.pos(3.16),10,0.1);
        TS_ASSERT_DELTA(stp3.vel(3.16),0,0.1);
    }
 };

#endif	/* _stp3TestSuite_H */


/**
 * \file stp3TestSuite.h
 * \author Erik Weitnauer
 */

#ifndef _stp3TestSuite_H
#define	_stp3TestSuite_H

#include <cxxtest/TestSuite.h>
#include "stp3.h"

/**
 * \brief Class for automated testing of the Stp3 class.
 * \author Erik Weitnauer
 * \date 2007
 *
 * The "cxxtest" testing system is used.
 * \see cxxtest.sourceforge.net/
 */
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
    
    void testScale( void ) {
    	Stp3 stp;
    	// canonical
        TS_ASSERT_DIFFERS(stp.toString(), "");
        TS_ASSERT_THROWS_NOTHING(stp.planFastestProfile(0, 30, 0, 6, 4));
        TS_ASSERT_DELTA(stp.getDuration(), 6.5 ,0.0001);
        TS_ASSERT_DELTA(stp.pos(8),30,0.0001);
        TS_ASSERT_DIFFERS(stp.toString(), "");
        
        TS_ASSERT_THROWS_NOTHING(stp.scaleToDuration(5));
        TS_ASSERT_DELTA(stp.getDuration(), 6.5, 0.0001);
        TS_ASSERT_DELTA(stp.pos(8),30,0.0001);
        
        TS_ASSERT_THROWS_NOTHING(stp.scaleToDuration(7));
        TS_ASSERT_DELTA(stp.getDuration(), 7, 0.0001);
        TS_ASSERT_DELTA(stp.pos(8),30,0.0001);
        
        // double dec
        TS_ASSERT_THROWS_NOTHING(stp.planFastestProfile(0, 30, 9, 6, 4));
        TS_ASSERT_DELTA(stp.getDuration(), 5.5625, 0.001);
        TS_ASSERT_DELTA(stp.pos(8),30,0.0001);
        
        TS_ASSERT_THROWS_NOTHING(stp.scaleToDuration(6));
        TS_ASSERT_DELTA(stp.getDuration(), 6, 0.001);
        TS_ASSERT_DELTA(stp.pos(8),30,0.0001);
        cout << endl << stp.toString() << endl;        
    }
 };

#endif	/* _stp3TestSuite_H */


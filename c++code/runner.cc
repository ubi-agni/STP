/* Generated file, do not edit */

#ifndef CXXTEST_RUNNING
#define CXXTEST_RUNNING
#endif

#define _CXXTEST_HAVE_STD
#define _CXXTEST_HAVE_EH
#include <cxxtest/TestListener.h>
#include <cxxtest/TestTracker.h>
#include <cxxtest/TestRunner.h>
#include <cxxtest/RealDescriptions.h>
#include <cxxtest/StdioPrinter.h>
#include <cxxtest/Win32Gui.h>

int main( int argc, char *argv[] ) {
 return CxxTest::GuiTuiRunner<CxxTest::Win32Gui, CxxTest::StdioPrinter>( argc, argv ).run();
}
#include "stp3TestSuite.h"

static Stp3TestSuite suite_Stp3TestSuite;

static CxxTest::List Tests_Stp3TestSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_Stp3TestSuite( "stp3TestSuite.h", 14, "Stp3TestSuite", suite_Stp3TestSuite, Tests_Stp3TestSuite );

static class TestDescription_Stp3TestSuite_testBasics : public CxxTest::RealTestDescription {
public:
 TestDescription_Stp3TestSuite_testBasics() : CxxTest::RealTestDescription( Tests_Stp3TestSuite, suiteDescription_Stp3TestSuite, 20, "testBasics" ) {}
 void runTest() { suite_Stp3TestSuite.testBasics(); }
} testDescription_Stp3TestSuite_testBasics;

#include "stp7TestSuite.h"

static Stp7TestSuite suite_Stp7TestSuite;

static CxxTest::List Tests_Stp7TestSuite = { 0, 0 };
CxxTest::StaticSuiteDescription suiteDescription_Stp7TestSuite( "stp7TestSuite.h", 14, "Stp7TestSuite", suite_Stp7TestSuite, Tests_Stp7TestSuite );

static class TestDescription_Stp7TestSuite_testBasics : public CxxTest::RealTestDescription {
public:
 TestDescription_Stp7TestSuite_testBasics() : CxxTest::RealTestDescription( Tests_Stp7TestSuite, suiteDescription_Stp7TestSuite, 16, "testBasics" ) {}
 void runTest() { suite_Stp7TestSuite.testBasics(); }
} testDescription_Stp7TestSuite_testBasics;

static class TestDescription_Stp7TestSuite_testCruiseProfilesStandard : public CxxTest::RealTestDescription {
public:
 TestDescription_Stp7TestSuite_testCruiseProfilesStandard() : CxxTest::RealTestDescription( Tests_Stp7TestSuite, suiteDescription_Stp7TestSuite, 27, "testCruiseProfilesStandard" ) {}
 void runTest() { suite_Stp7TestSuite.testCruiseProfilesStandard(); }
} testDescription_Stp7TestSuite_testCruiseProfilesStandard;

static class TestDescription_Stp7TestSuite_testCruiseProfilesSpecial : public CxxTest::RealTestDescription {
public:
 TestDescription_Stp7TestSuite_testCruiseProfilesSpecial() : CxxTest::RealTestDescription( Tests_Stp7TestSuite, suiteDescription_Stp7TestSuite, 58, "testCruiseProfilesSpecial" ) {}
 void runTest() { suite_Stp7TestSuite.testCruiseProfilesSpecial(); }
} testDescription_Stp7TestSuite_testCruiseProfilesSpecial;

static class TestDescription_Stp7TestSuite_testNoCruiseProfilesStandard : public CxxTest::RealTestDescription {
public:
 TestDescription_Stp7TestSuite_testNoCruiseProfilesStandard() : CxxTest::RealTestDescription( Tests_Stp7TestSuite, suiteDescription_Stp7TestSuite, 82, "testNoCruiseProfilesStandard" ) {}
 void runTest() { suite_Stp7TestSuite.testNoCruiseProfilesStandard(); }
} testDescription_Stp7TestSuite_testNoCruiseProfilesStandard;

static class TestDescription_Stp7TestSuite_testNoCruiseProfileDoubleDec : public CxxTest::RealTestDescription {
public:
 TestDescription_Stp7TestSuite_testNoCruiseProfileDoubleDec() : CxxTest::RealTestDescription( Tests_Stp7TestSuite, suiteDescription_Stp7TestSuite, 117, "testNoCruiseProfileDoubleDec" ) {}
 void runTest() { suite_Stp7TestSuite.testNoCruiseProfileDoubleDec(); }
} testDescription_Stp7TestSuite_testNoCruiseProfileDoubleDec;

#include <cxxtest/Root.cpp>

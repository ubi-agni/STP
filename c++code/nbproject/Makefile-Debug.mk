#
# Gererated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add custumized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=build/Debug/GNU-Linux-x86

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/polynomial.o \
	${OBJECTDIR}/stp7Formulars.o \
	${OBJECTDIR}/stp7.o \
	${OBJECTDIR}/complex.o \
	${OBJECTDIR}/stp3.o \
	${OBJECTDIR}/tools.o \
	${OBJECTDIR}/runner.o

# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS} dist/Debug/GNU-Linux-x86/stp7

dist/Debug/GNU-Linux-x86/stp7: ${OBJECTFILES}
	${MKDIR} -p dist/Debug/GNU-Linux-x86
	${LINK.cc} -o dist/Debug/GNU-Linux-x86/stp7 ${OBJECTFILES} ${LDLIBSOPTIONS} 

${OBJECTDIR}/polynomial.o: polynomial.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/polynomial.o polynomial.cc

${OBJECTDIR}/stp7Formulars.o: stp7Formulars.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/stp7Formulars.o stp7Formulars.cc

${OBJECTDIR}/stp7.o: stp7.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/stp7.o stp7.cc

${OBJECTDIR}/complex.o: complex.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/complex.o complex.cc

${OBJECTDIR}/stp3.o: stp3.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/stp3.o stp3.cc

${OBJECTDIR}/tools.o: tools.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/tools.o tools.cc

${OBJECTDIR}/runner.o: runner.cc 
	${MKDIR} -p ${OBJECTDIR}
	$(COMPILE.cc) -g -Icxxtest -o ${OBJECTDIR}/runner.o runner.cc

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf:
	${RM} -r build/Debug
	${RM} dist/Debug/GNU-Linux-x86/stp7

# Subprojects
.clean-subprojects:

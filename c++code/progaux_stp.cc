#include "MultiSTP.hh"

extern "C" {
#include <nst.h>
#include <nst_prog.h>
#include <nst_prog_ops.h>

#define SMALL_EPSILON  1.0e-6
char ARRAYDIM_SHOULD_BE[] = "%s: dim of array arg #%d must be %d\n";

// {{{  MultiSTP : 

const char *MultiSTP_c[] = {
  "STP_:", // class prefix
  "STP (int nAxes, int b3rdOrder)", // class constructor
  "~STP ()", // class destructor
  
  "float planFastestProfile (float* pfGoal, float* pfCur, "
  "                          float* pfVel=NULL, float* pfAcc=NULL)",
  "void  scaleToDuration (float fNewDuration)",
  "int   isAfterCruising (float t)",
  "float getDuration ()",

  "void  setMax    (float* vMaxVel, float* vMaxAcc, float* vMaxJer=NULL)",
  "void  setSynced (int iSync)",

  "void  getPos (float t, float *pfPos)",
  "void  getVel (float t, float *pfVel)",
  "void  getAcc (float t, float *pfAcc)",
  "void  getJer (float t, float *pfAcc)",
  NULL
};


// class constructor
MultiSTP* STP_STP (int *piDim, void **ppvArg) {
   int iOrder = *((int*)ppvArg[1]);
   return new MultiSTP (*((int*)ppvArg[0]), 
                        iOrder != 0 && iOrder != 3);
}
// class destructor
void STP_FREE_STP (MultiSTP* pSTP) {
   if (pSTP) delete pSTP;
}

float STP_planFastestProfile (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   int N = pSTP->getNumAxes ();
   if (piDim[0] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 0, N);
   if (piDim[1] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 1, N);
   if (ppfArg[2] && piDim[2] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 2, N);

   MultiSTP::ValueArray vVel (N), vAcc(N),
      vGoal    (ppfArg[0], ppfArg[0] + N),
      vCurrent (ppfArg[1], ppfArg[1] + N);
   if (ppfArg[2]) std::copy (ppfArg[2], ppfArg[2] + N, vVel.begin());
   if (ppfArg[3]) std::copy (ppfArg[3], ppfArg[3] + N, vAcc.begin());
   
   if (ppfArg[3])
      return pSTP->planFastestProfile (vGoal, vCurrent, vVel, vAcc);
   else
      return pSTP->planFastestProfile (vGoal, vCurrent, vVel);
}

void STP_scaleToDuration (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   if (!pSTP->scaleToDuration (*ppfArg[0]))
      prog_throw ("badarg", "cannot shorten duration");
}

int STP_isAfterCruising (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   return pSTP->isAfterCruising (*ppfArg[0]);
}

float STP_getDuration (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   return pSTP->getDuration ();
}

void STP_setMax (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   int N = pSTP->getNumAxes ();
   if (piDim[0] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 0, N);
   if (piDim[1] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 1, N);
   if (piDim[2] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 2, N);
   
   MultiSTP::ValueArray vMaxJer(N, 1),
      vMaxVel (ppfArg[0], ppfArg[0] + N),
      vMaxAcc (ppfArg[1], ppfArg[1] + N);
   if (ppfArg[2]) std::copy (ppfArg[2], ppfArg[2] + N, vMaxJer.begin());

   for (int i=0; i < N; i++) {
      if (vMaxVel[i] < SMALL_EPSILON || 
          vMaxAcc[i] < SMALL_EPSILON ||
          vMaxJer[i] < SMALL_EPSILON)
         prog_throw ("badarg", "max values too small (%d)", i);
   }
   if (ppfArg[2]) pSTP->setMax (vMaxVel, vMaxAcc, vMaxJer);
   else pSTP->setMax (vMaxVel, vMaxAcc);
}
void STP_setSynced (int *piDim, void **ppvArg, MultiSTP *pSTP) {
   pSTP->setSyncMode ((*(int*)ppvArg[0]) 
                      ? MultiSTP::synced : MultiSTP::fastest);
}

void STP_getPos (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   int N = pSTP->getNumAxes ();
   if (piDim[1] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 0, N);

   const MultiSTP::ValueArray& vVal = pSTP->pos (*ppfArg[0]);
   std::copy (vVal.begin(), vVal.end(), ppfArg[1]);
}
void STP_getVel (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   int N = pSTP->getNumAxes ();
   if (piDim[1] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 0, N);

   const MultiSTP::ValueArray& vVal = pSTP->vel (*ppfArg[0]);
   std::copy (vVal.begin(), vVal.end(), ppfArg[1]);
}
void STP_getAcc (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   int N = pSTP->getNumAxes ();
   if (piDim[1] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 0, N);

   const MultiSTP::ValueArray& vVal = pSTP->acc (*ppfArg[0]);
   std::copy (vVal.begin(), vVal.end(), ppfArg[1]);
}
void STP_getJer (int *piDim, float **ppfArg, MultiSTP *pSTP) {
   int N = pSTP->getNumAxes ();
   if (piDim[1] != N) 
      prog_throw (BADARG_ARRAY_DIM, ARRAYDIM_SHOULD_BE, __FUNCTION__, 0, N);

   const MultiSTP::ValueArray& vVal = pSTP->jerk (*ppfArg[0]);
   std::copy (vVal.begin(), vVal.end(), ppfArg[1]);
}

// }}} 

const char *NAMESPACE = "stp";
const char **CLASSES_stp[] = {MultiSTP_c, NULL};

}

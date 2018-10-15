#ifndef C_MOOSE_H
#define C_MOOSE_H

/*****************************************************************************
 BEGIN: Copied from cFullModel_Oct11:339..449. Changes:

* "int" ->
  "long"

* only leave
  #define M_INT long

* remove the definition of EVTHYST

  #define EVTHYST 1.000000e-10
*****************************************************************************/

/* Configurable parameters */
#define CONITER 30
#define CONTOL 1.000000e-05
#define INITCONITER 100
#define INITCONTOL 1.000000e-10
#define INITITER 50
#define INITTOL 1.000000e-08
#define INITWEIGHT 2.000000e+01
#define EVTITER 10
#define EVTPROJ 1
#define EVTHYST 1.000000e-10
#define INCONTOL 1e200

#define M_INT long

/* Fixed parameters */
#define NDIFF 64
#define NDFA 64
#define NSDIFF 64
#define NEQ 457
#define NPAR 27
#define NINP 3
#define NDISC 126
#define NIX1 267
#define NOUT 122
#define NCON 2
#define NEVT 80
#ifdef EVTHYST
#define NZC 2*NEVT
#else
#define NZC NEVT
#endif

typedef struct {
	double h;		/* Integration step size */
	double *w;		/* Float workspace */
	long *iw;		/* Integer workspace */
	long err;			/* Error flag */
	char *buf;		/* Error message */
} SolverStruct;

/*****************************************************************************
 END: Copied from cFullModel_Oct11:339..449
*****************************************************************************/

#endif  // C_MOOSE_H

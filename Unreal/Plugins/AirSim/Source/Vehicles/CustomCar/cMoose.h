#ifndef C_MOOSE_H
#define C_MOOSE_H

// BEGIN: Copied from cFullModel_Sept26:413..437

/* Fixed parameters */
#define NDIFF 64
#define NDFA 64
#define NSDIFF 64
#define NEQ 445
#define NPAR 27
#define NINP 3
#define NDISC 126
#define NIX1 255
#define NOUT 110
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

// BEGIN: Copied from cFullModel_Sept26:413..437

#endif  // C_MOOSE_H

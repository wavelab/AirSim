#ifndef VSM_H
#define VSM_H

#define NEQ 386
#define NPAR 27
#define NINP 3
#define NOUT 113
#define NEVT 72
#define NDFA 64

typedef struct {
	double h;		/* Integration step size */
	double *w;		/* Float workspace */
	long *iw;		/* Integer workspace */
	long err;		/* Error flag */
	char *buf;		/* Error message */
} SolverStruct;

#endif  // VSM_H

/*
 * levmarq.c
 *
 * This file contains an implementation of the Levenberg-Marquardt algorithm
 * for solving least-squares problems, together with some supporting routines
 * for Cholesky decomposition and inversion.  No attempt has been made at
 * optimization.  In particular, memory use in the matrix routines could be
 * cut in half with a little effort (and some loss of clarity).
 *
 * It is assumed that the compiler supports variable-length arrays as
 * specified by the C99 standard.
 *
 * Ron Babich, May 2008
 *
 */

#include <stdio.h>
#include <math.h>
#include "levmarq.h"

//#define PRINT_DEBUG

#define TOL 1e-30 /* smallest value allowed in cholesky_decomp() */


/* set parameters required by levmarq() to default values */
void levmarq_init(LMstat *lmstat)
{
  lmstat->verbose = 0;
  lmstat->max_it = 10000;
  lmstat->init_lambda = 0.1;
  lmstat->up_factor = 10;
  lmstat->down_factor = 10;
  lmstat->target_derr = 0.01;
}


/* perform least-squares minimization using the Levenberg-Marquardt
   algorithm.  The arguments are as follows:

   npar    number of parameters
   par     array of parameters to be varied
   ny      number of measurements to be fit
   y       array of measurements
   dysq    array of error in measurements, squared
           (set dysq=NULL for unweighted least-squares)
   func    function to be fit
   grad    gradient of "func" with respect to the input parameters
   fdata   pointer to any additional data required by the function
   lmstat  pointer to the "status" structure, where minimization parameters
           are set and the final status is returned.

   Before calling levmarq, several of the parameters in lmstat must be set.
   For default values, call levmarq_init(lmstat).
 */
int levmarq(int npar, float *par, int ny, float *y, float *dysq,
	    float (*func)(float *, int, void *),
	    void (*grad)(float *, float *, int, void *),
	    void *fdata, LMstat *lmstat)
{
  int x,i,j,it,nit,ill,verbose;
  float lambda,up,down,mult,weight,err,newerr,derr,target_derr;
  float h[npar][npar],ch[npar][npar];
  float g[npar],d[npar],delta[npar],newpar[npar];

  verbose = lmstat->verbose;
  nit = lmstat->max_it;
  lambda = lmstat->init_lambda;
  up = lmstat->up_factor;
  down = 1/lmstat->down_factor;
  target_derr = lmstat->target_derr;
  weight = 1;
  derr = newerr = 0; /* to avoid compiler warnings */

  /* calculate the initial error ("chi-squared") */
  err = error_func(par, ny, y, dysq, func, fdata);

  /* main iteration */
  for (it=0; it<nit; it++) {

    /* calculate the approximation to the Hessian and the "derivative" d */
    for (i=0; i<npar; i++) {
      d[i] = 0;
      for (j=0; j<=i; j++)
	h[i][j] = 0;
    }
    for (x=0; x<ny; x++) {
      if (dysq) weight = 1/dysq[x]; /* for weighted least-squares */
      grad(g, par, x, fdata);
      for (i=0; i<npar; i++) {
	d[i] += (y[x] - func(par, x, fdata))*g[i]*weight;
	for (j=0; j<=i; j++)
	  h[i][j] += g[i]*g[j]*weight;
      }
    }

    /*  make a step "delta."  If the step is rejected, increase
       lambda and try again */
    mult = 1 + lambda;
    ill = 1; /* ill-conditioned? */
    while (ill && (it<nit)) {
      for (i=0; i<npar; i++)
	h[i][i] = h[i][i]*mult;

      ill = cholesky_decomp(npar, ch, h);

      if (!ill) {
	solve_axb_cholesky(npar, ch, delta, d);
	for (i=0; i<npar; i++)
	  newpar[i] = par[i] + delta[i];
	newerr = error_func(newpar, ny, y, dysq, func, fdata);
	derr = newerr - err;
	ill = (derr > 0);
      } 
      //if (verbose) trace_printf("it = %4d,   lambda = %10g,   err = %10g,   "
	//		  "derr = %10g\n", it, lambda, err, derr);
      if (ill) {
	mult = (1 + lambda*up)/(1 + lambda);
	lambda *= up;
	it++;
      }
    }
    for (i=0; i<npar; i++)
      par[i] = newpar[i];
    err = newerr;
    lambda *= down;  
#ifdef PRINT_DEBUG
    trace_printf("Iteration %d: ", it);
    for (i = 0; i < npar; i++)
    {
    	trace_printf("%f:",par[i]);
    }
    trace_printf("Error: %f\n", derr);
#endif
    if ((!ill)&&(-derr<target_derr))
    {
#ifdef PRINT_DEBUG
    	trace_printf("Converge after: %d cycles, final error: %f\n", it, derr);
#endif
        break;
    }
  }

  lmstat->final_it = it;
  lmstat->final_err = err;
  lmstat->final_derr = derr;

  return it;
}


/* calculate the error function (chi-squared) */
float error_func(float *par, int ny, float *y, float *dysq,
		  float (*func)(float *, int, void *), void *fdata)
{
  int x;
  float res,e=0;

  for (x=0; x<ny; x++) {
    res = func(par, x, fdata) - y[x];
    if (dysq)  /* weighted least-squares */
      e += res*res/dysq[x];
    else
      e += res*res;
  }
  return e;
}


/* solve the equation Ax=b for a symmetric positive-definite matrix A,
   using the Cholesky decomposition A=LL^T.  The matrix L is passed in "l".
   Elements above the diagonal are ignored.
*/
void solve_axb_cholesky(int n, float l[n][n], float x[n], float b[n])
{
  int i,j;
  float sum;

  /* solve L*y = b for y (where x[] is used to store y) */

  for (i=0; i<n; i++) {
    sum = 0;
    for (j=0; j<i; j++)
      sum += l[i][j] * x[j];
    x[i] = (b[i] - sum)/l[i][i];      
  }

  /* solve L^T*x = y for x (where x[] is used to store both y and x) */

  for (i=n-1; i>=0; i--) {
    sum = 0;
    for (j=i+1; j<n; j++)
      sum += l[j][i] * x[j];
    x[i] = (x[i] - sum)/l[i][i];      
  }
}


/* This function takes a symmetric, positive-definite matrix "a" and returns
   its (lower-triangular) Cholesky factor in "l".  Elements above the
   diagonal are neither used nor modified.  The same array may be passed
   as both l and a, in which case the decomposition is performed in place.
*/
int cholesky_decomp(int n, float l[n][n], float a[n][n])
{
  int i,j,k;
  float sum;

  for (i=0; i<n; i++) {
    for (j=0; j<i; j++) {
      sum = 0;
      for (k=0; k<j; k++)
	sum += l[i][k] * l[j][k];
      l[i][j] = (a[i][j] - sum)/l[j][j];
    }

    sum = 0;
    for (k=0; k<i; k++)
      sum += l[i][k] * l[i][k];
    sum = a[i][i] - sum;
    if (sum<TOL) return 1; /* not positive-definite */
    l[i][i] = sqrtf(sum);
  }
  return 0;
}

typedef struct {
  int verbose;
  int max_it;
  float init_lambda;
  float up_factor;
  float down_factor;
  float target_derr;
  int final_it;
  float final_err;
  float final_derr;
} LMstat;

void levmarq_init(LMstat *lmstat);

int levmarq(int npar, float *par, int ny, float *y, float *dysq,
	    float (*func)(float *, int, void *),
	    void (*grad)(float *, float *, int, void *),
	    void *fdata, LMstat *lmstat);

float error_func(float *par, int ny, float *y, float *dysq,
		  float (*func)(float *, int, void *), void *fdata);

void solve_axb_cholesky(int n, float l[n][n], float x[n], float b[n]);

int cholesky_decomp(int n, float l[n][n], float a[n][n]);

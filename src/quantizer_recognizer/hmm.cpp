#include "hmm.h"
#include <iostream>
using std::cout;
void ReadHMM(FILE *fp, HMM *phmm)
{
	int i, j, k;

	fscanf(fp, "M= %d\n", &(phmm->M)); 

	fscanf(fp, "N= %d\n", &(phmm->N)); 

	fscanf(fp, "A:\n");
	phmm->A = (double **) dmatrix(1, phmm->N, 1, phmm->N);
	for (i = 1; i <= phmm->N; i++) { 
		for (j = 1; j <= phmm->N; j++) {
			fscanf(fp, "%lf", &(phmm->A[i][j])); 
		}
		fscanf(fp,"\n");
	}

	fscanf(fp, "B:\n");
	phmm->B = (double **) dmatrix(1, phmm->N, 1, phmm->M);
	for (j = 1; j <= phmm->N; j++) { 
		for (k = 1; k <= phmm->M; k++) {
			fscanf(fp, "%lf", &(phmm->B[j][k])); 
		}
		fscanf(fp,"\n");
	}

	fscanf(fp, "pi:\n");
	phmm->pi = (double *) dvector(1, phmm->N);
	for (i = 1; i <= phmm->N; i++) 
		fscanf(fp, "%lf", &(phmm->pi[i])); 
}

double **dmatrix(int nrl,int nrh,int ncl,int nch)
{
	int i;
	double **m;

	m=(double **) calloc((unsigned) (nrh-nrl+1),sizeof(double*));
	if (!m) nrerror("allocation failure 1 in dmatrix()");
	m -= nrl;

	for(i=nrl;i<=nrh;i++) {
		m[i]=(double *) calloc((unsigned) (nch-ncl+1),sizeof(double));
		if (!m[i]) nrerror("allocation failure 2 in dmatrix()");
		m[i] -= ncl;
	}
	return m;
}


double *dvector(int nl, int nh)
{
	double *v;

	v=(double *)calloc((unsigned) (nh-nl+1),sizeof(double));
	if (!v) nrerror("allocation failure in dvector()");
	return v-nl;
}

void nrerror(char error_text[])
{

	fprintf(stderr,"Numerical Recipes run-time error...\n");
	fprintf(stderr,"%s\n",error_text);
	fprintf(stderr,"...now exiting to system...\n");
	exit(1);
}


void Forward(HMM *phmm, int T, int *O, double **alpha, double *pprob)
{
        int     i, j;   /* state indices */
        int     t;      /* time index */
 
        double sum;     /* partial sum */
 	
        /* 1. Initialization */
        
        for (i = 1; i <= phmm->N; i++){
                alpha[1][i] = phmm->pi[i]* phmm->B[i][O[0]];	
		//cout<<"pi[i]*B[i][O[1]]="<<phmm->pi[i]<<"*"<<phmm->B[i][O[1]]<<"="<<alpha[1][i]<<"\n";	        
	}
	
        /* 2. Induction */
 
        for (t = 1; t < T; t++) {
                for (j = 1; j <= phmm->N; j++) {
                        sum = 0.0;
                        for (i = 1; i <= phmm->N; i++)
                                sum += alpha[t][i]* (phmm->A[i][j]);
 
                        alpha[t+1][j] = sum*(phmm->B[j][O[t]]);
			//cout<<"alpha[i][j]=sum*B[j][O[t]]"<<sum<<"*"<<phmm->B[j][O[t]]<<"="<<alpha[t+1][j]<<"\n";
                }
        }
 
        /* 3. Termination */
        *pprob = 0.0;
        for (i = 1; i <= phmm->N; i++){
                *pprob += alpha[T][i];
		//cout <<"log(*pprob) "<<log(*pprob); 	
	}
}

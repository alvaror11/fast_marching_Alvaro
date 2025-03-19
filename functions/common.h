#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define eps 2.2204460492503131e-16
#define doublemax 1e50
#define INF 2e50
#define listINF 2.345e50

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

/* Function declarations */
int minarray(double *A, int l);
int maxarray(double *A, int l);
double pow2(double val);
int iszero(double a);
int isnotzero(double a);
void roots(double *Coeff, double *ans);
int p2x(int x);
void show_list(double **listval, int *listprop);
void initialize_list(double **listval, int *listprop);
void destroy_list(double **listval, int *listprop);
void list_add(double **listval, int *listprop, double val);
int list_minimum(double **listval, int *listprop);
void list_remove(double **listval, int *listprop, int index);
void list_remove_replace(double **listval, int *listprop, int index);
void listupdate(double **listval, int *listprop, int index, double val);

int mindex3(int x, int y, int z, int sizx, int sizy) ;

bool IsFinite(double x);

bool IsInf(double x);

bool IsListInf(double x);

bool isntfrozen3d(int i, int j, int k, int *dims, bool *Frozen);

bool isfrozen3d(int i, int j, int k, int *dims, bool *Frozen);

int mindex2(int x, int y, int sizx);

bool isntfrozen2d(int i, int j, int *dims, bool *Frozen);

bool isfrozen2d(int i, int j, int *dims, bool *Frozen);

void print_memory_usage(const char* checkpoint);

#endif // COMMON_H
#ifndef COMMON_H
#define COMMON_H

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#define eps 2.2204460492503131e-16
#define floatmax 1e50
#define INF 2e50
#define listINF 2.345e50

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

/* Function declarations */
int minarray(float *A, int l);
int maxarray(float *A, int l);
float pow2(float val);
int iszero(float a);
int isnotzero(float a);
void roots(float *Coeff, float *ans);
int p2x(int x);
void show_list(float **listval, int *listprop);
void initialize_list(float **listval, int *listprop);
void destroy_list(float **listval, int *listprop);
void list_add(float **listval, int *listprop, float val);
int list_minimum(float **listval, int *listprop);
void list_remove(float **listval, int *listprop, int index);
void list_remove_replace(float **listval, int *listprop, int index);
void listupdate(float **listval, int *listprop, int index, float val);

int mindex3(int x, int y, int z, int sizx, int sizy) ;

bool IsFinite(float x);

bool IsInf(float x);

bool IsListInf(float x);

bool isntfrozen3d(int i, int j, int k, int *dims, bool *Frozen);

bool isfrozen3d(int i, int j, int k, int *dims, bool *Frozen);

int mindex2(int x, int y, int sizx);

bool isntfrozen2d(int i, int j, int *dims, bool *Frozen);

bool isfrozen2d(int i, int j, int *dims, bool *Frozen);

void print_memory_usage(const char* checkpoint);

float euclidean_distance(int x1, int y1, int x2, int y2);

float euclidean_distance3D(int x1, int y1, int z1, int x2, int y2, int z2);

void read_map(float* matriz, int* size_map, char* mapfile, int dimensions_prob);

float* process_map_file(char* mapfile, int* size_map, int dimensions_prob);

#endif // COMMON_H
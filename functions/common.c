#define eps 2.2204460492503131e-16
#define floatmax 1e50
#define INF 2e50
#define listINF 2.345e50
#ifndef min
#define min(a,b)        ((a) < (b) ? (a): (b))
#endif
#ifndef max
#define max(a,b)        ((a) > (b) ? (a): (b))
#endif

#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "common.h"
#include <windows.h>
#include <psapi.h>


void print_memory_usage(const char* checkpoint) {
    PROCESS_MEMORY_COUNTERS_EX pmc;
    if (GetProcessMemoryInfo(GetCurrentProcess(), (PROCESS_MEMORY_COUNTERS*)&pmc, sizeof(pmc))) {
        float working_set = (float)pmc.WorkingSetSize / (1024*1024);
        float private_usage = (float)pmc.PrivateUsage / (1024*1024);
        
        // Add validation and detailed information
        FILE* mem_log = fopen("./Archivos/memory_log.txt", "a");
        if (mem_log != NULL) {
            fprintf(mem_log, "=== Memory Checkpoint: %s ===\n", checkpoint);
            fprintf(mem_log, "Working Set: %.2f MB\n", working_set);
            fprintf(mem_log, "Private Usage: %.2f MB\n", private_usage);
            
            // Detailed memory information
            fprintf(mem_log, "Peak Working Set: %.2f MB\n", 
                   (float)pmc.PeakWorkingSetSize / (1024*1024));
            fprintf(mem_log, "Page File Usage: %.2f MB\n", 
                   (float)pmc.PagefileUsage / (1024*1024));
            fprintf(mem_log, "Peak Page File: %.2f MB\n", 
                   (float)pmc.PeakPagefileUsage / (1024*1024));
            
            // Warning for unusual conditions
            if (working_set > private_usage) {
                fprintf(mem_log, "WARNING: Working Set (%.2f MB) exceeds Private Usage (%.2f MB)\n",
                        working_set, private_usage);
                fprintf(mem_log, "This may indicate a system-level issue or shared memory usage\n");
            }
            
            fprintf(mem_log, "==============================\n\n");
            fclose(mem_log);
        }
    }
}

/* Find minimum value of an array and return its index */
int minarray(float *A, int l) {
    int i;
    int minind=0;
    for (i=0; i<l; i++) { if(A[i]<A[minind]){ minind=i; } }
    return minind;
}

float pow2(float val) { return val*val; }

int iszero(float a){ return a*a<eps; }
int isnotzero(float a){ return a*a>eps; }

void roots(float* Coeff, float* ans) {
    float a=Coeff[0];
    float b=Coeff[1];
    float c=Coeff[2];
    float r1, r2;
    float d;
    
    d=max(pow2(b)-4.0*a*c,0);
    if(isnotzero(a)) {
        ans[0]= (-b - sqrt(d)) / (2.0*a);
        ans[1]= (-b + sqrt(d)) / (2.0*a);
    }
    else {
		r1=(-b - sqrt(d));
		r2=(-b + sqrt(d));
		if(isnotzero(r1))
		{
			if(isnotzero(r2))
			{
				ans[0]= (2.0*c)/r1; ans[1]= (2.0*c)/r2;
			}
			else
			{
				ans[0]= (2.0*c)/r1; ans[1]= (2.0*c)/r1;
			}
		}
		else if(isnotzero(r2))
		{
			ans[0]= (2.0*c)/r2; ans[1]= (2.0*c)/r2;
		}
		else
		{
			ans[0]=0; ans[1]=0;
		}
    }
}


int maxarray(float *A, int l) {
    int i;
    int maxind=0;
    for (i=0; i<l; i++) { if(A[i]>A[maxind]){ maxind=i; } }
    return maxind;
}


int p2x(int x) /* 2^x */
{
/*    return pow(2,x); */
    int y=1;
    int p2x[16] ={1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
    while(x>15) { x=x-15; y=y*32768; }
    return y*p2x[x];
}

void show_list(float **listval, int *listprop) {
    int z, k;
    for(z=0;z<listprop[1]; z++) {
        for(k=0;k<p2x(z+1); k++) {
            if((z>0)&&(listval[z-1][(int)floor(k/2)]>listval[z][k])) {
                printf("*%15.5f", listval[z][k]);
            }
            else {
                printf(" %15.5f", listval[z][k]);
            }
        }
        printf("\n");
    }
}

void initialize_list(float ** listval, int *listprop) {
    /* Loop variables */
    int i;
    /* Current Length, Orde and Current Max Length */
    listprop[0]=0; listprop[1]=1; listprop[2]=2;
    /* Make first orde storage of 2 values */
    listval[0]=(float*)malloc(2 * sizeof(float));
    /* Initialize on infinite */
    for(i=0;i<2;i++) { listval[0][i]=listINF; }
}


void destroy_list(float ** listval, int *listprop) {
    /* Loop variables */
    int i, list_orde;
    /* Get list orde */
    list_orde=listprop[1];
    /* Free memory */
    for(i=0;i<list_orde;i++) { free(listval[i]); }
    free(listval);
    free(listprop);
}

void list_add(float ** listval, int *listprop, float val) {
    /* List parameters */
    int list_length, list_orde, list_lengthmax;
    /* Loop variable */
    int i, j;
    /* Temporary list location */
    int listp;
    /* Get list parameters */
    list_length=listprop[0]; list_orde=listprop[1]; list_lengthmax=listprop[2];
    /* If list is full expand list */
    if(list_length==list_lengthmax) {
        list_lengthmax=list_lengthmax*2;
        for (i=list_orde; i>0; i--) {
            listval[i]=listval[i-1];
            listval[i] = (float *)realloc(listval[i], p2x(i+1)*sizeof(float));
            for(j=p2x(i); j<p2x(i+1); j++) { listval[i][j]=listINF;  }
        }
        listval[0]=(float *)malloc(2*sizeof(float));
        listval[0][0]=min(listval[1][0], listval[1][1]);
        listval[0][1]=listINF;
        list_orde++;
    }
    /* Add a value to the list */
    listp=list_length;
    list_length++;
    listval[list_orde-1][listp]=val;
    /* Update the links minimum */
    for (i=list_orde-1; i>0; i--) {
        listp=(int)floor(((float)listp)/2);
        if(val<listval[i-1][listp]) { listval[i-1][listp]=val; } else { break; }
    }
    /* Set list parameters */
    listprop[0]=list_length; listprop[1]=list_orde; listprop[2]=list_lengthmax;
}

int list_minimum(float ** listval, int *listprop) {
    /* List parameters */
    int list_length, list_orde, list_lengthmax;
    /* Loop variable */
    int i;
    /* Temporary list location */
    int listp;
    /* Index of Minimum */
    int minindex;
    /* Get list parameters */
    list_length=listprop[0]; list_orde=listprop[1]; list_lengthmax=listprop[2];
    /* Follow the minimum through the binary tree */
    listp=0;
    for(i=0;i<(list_orde-1);i++) {
        
        /* <= ????? */ 
        if(listval[i][listp]<=listval[i][listp+1]) { listp=listp*2; } else { listp=(listp+1)*2; }
    }
    i=list_orde-1;
    if(listval[i][listp]<=listval[i][listp+1]){minindex=listp; } else { minindex=listp+1; }
    return minindex;
}
void list_remove(float ** listval, int *listprop, int index) {
    /* List parameters */
    int list_length, list_orde, list_lengthmax;
    /* Loop variable */
    int i;
    /* Temp index */
    int index2;
    float val;
    float valmin;
    /* Get list parameters */
    list_length=listprop[0];
    list_orde=listprop[1];
    list_lengthmax=listprop[2];
    /* Temporary store current value */
    val=listval[list_orde-1][index];
    valmin=listINF;
    /* Replace value by infinite */
    listval[list_orde-1][index]=listINF;
    /* Follow the binary tree to replace value by minimum values from */
    /** the other values in the binary tree. */

    i=list_orde-1;
    while(true) {
        if((index%2)==0) { index2=index+1; } else { index2=index-1; }
        if(val<listval[i][index2]) {
            index=(int)floor(((float)index2)/2.0);
            if(listval[i][index2]<valmin) { valmin=listval[i][index2]; }
			if(i==0) { break; }
            listval[i-1][index]=valmin;
            i--; if(i==0) { break; }
        }
        else { break; }
    }
}

void list_remove_replace(float ** listval, int *listprop, int index) {
    /* List parameters */
    int list_length, list_orde, list_lengthmax;
    /* Loop variable */
    int i, listp;
    /* Temporary store value */
    float val;
    int templ;
    /* Get list parameters */
    list_length=listprop[0];
    list_orde=listprop[1];
    list_lengthmax=listprop[2];
    /* Remove the value */
    list_remove(listval, listprop, index);
    /* Replace the removed value by the last in the list. (if it was */
    /* not already the last value) */
    if(index<(list_length-1)) {
        /* Temporary store last value in the list */
        val=listval[list_orde-1][list_length-1];
        /* Remove last value in the list */
        list_remove(listval, listprop, list_length-1);
        /* Add a value to the list */
        listp=index;
        listval[list_orde-1][index]=val;
        /* Update the links minimum */
        for (i=(list_orde-1); i>0; i--) {
            listp=(int)floor(((float)listp)/2);
            if(val<listval[i-1][listp]) { listval[i-1][listp]=val; } else {  break; }
        }
    }
    /* List is now shorter */
    list_length--;
    /* Remove trunk of binary tree  / Free memory if list becomes shorter */
    if(list_orde>2&&IsListInf(listval[0][1])) {
        list_orde--;
        list_lengthmax=list_lengthmax/2;
        /* Remove trunk array */
        free(listval[0]);
        /* Move the other arrays one up */
        templ=2;
        for (i=0; i<list_orde; i++) {
            listval[i]=listval[i+1];
            /* Resize arrays to their new shorter size */
            listval[i] = (float *)realloc(listval[i], templ*sizeof(float));
            templ*=2;
        }
    }
    /* Set list parameters */
    listprop[0]=list_length; listprop[1]=list_orde; listprop[2]=list_lengthmax;
}

void listupdate(float **listval, int *listprop, int index, float val) {
    /* List parameters */
    int list_length, list_orde, list_lengthmax;
    /* loop variable */
    int i, listp;
    /* Get list parameters */
    list_length=listprop[0];
    list_orde=listprop[1];
    list_lengthmax=listprop[2];
    /* Add a value to the list */
    listp=index;
    listval[list_orde-1][index]=val;
    /* Update the links minimum */
    for (i=(list_orde-1); i>0; i--) {
        listp=(int)floor(((float)listp)/2);
        if(val<listval[i-1][listp]) { listval[i-1][listp]=val; } else { break; }
    }
    /* Set list parameters */
    listprop[0]=list_length; listprop[1]=list_orde; listprop[2]=list_lengthmax;
}

int mindex3(int x, int y, int z, int sizx, int sizy) { 
    return y + x * sizy + z * sizx * sizy; 
}

int mindex2(int x, int y, int sizx) { 
    return x + y * sizx; 
}
bool IsFinite(float x) { 
    return (x <= floatmax && x >= -floatmax); 
}

bool IsInf(float x) { 
    return (x >= floatmax); 
}

bool IsListInf(float x) { 
    return (x == listINF); 
}

bool isntfrozen3d(int i, int j, int k, int *dims, bool *Frozen) {
    return (i >= 0) && (j >= 0) && (k >= 0) &&
           (i < dims[0]) && (j < dims[1]) && (k < dims[2]) &&
           (Frozen[mindex3(i, j, k, dims[0], dims[1])] == 0);
}

bool isfrozen3d(int i, int j, int k, int *dims, bool *Frozen) {
    return (i >= 0) && (j >= 0) && (k >= 0) &&
           (i < dims[0]) && (j < dims[1]) && (k < dims[2]) &&
           (Frozen[mindex3(i, j, k, dims[0], dims[1])] == 1);
}


bool isntfrozen2d(int i, int j, int *dims, bool *Frozen) {
    return (i >= 0) && (j >= 0) && (i < dims[0]) && (j < dims[1]) && 
           (Frozen[i + j * dims[0]] == 0);
}

bool isfrozen2d(int i, int j, int *dims, bool *Frozen) {
    return (i >= 0) && (j >= 0) && (i < dims[0]) && (j < dims[1]) && 
           (Frozen[i + j * dims[0]] == 1);
}

float euclidean_distance(int x1, int y1, int x2, int y2) {
    float dx = (float)(x2 - x1);
    float dy = (float)(y2 - y1);
    return sqrt(dx*dx + dy*dy);
}

float euclidean_distance3D(int x1, int y1, int z1, int x2, int y2, int z2){
    float dx = (float)(x2 - x1);
    float dy = (float)(y2 - y1);
    float dz = (float)(z2 - z1);
    return sqrt(dx*dx + dy*dy + dz*dz);
}
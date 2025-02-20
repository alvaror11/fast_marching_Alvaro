#include "math.h"
#include "rk4_2D.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "common.h"

/* RK4 is a function which performs one step of the Runge-Kutta 4 ray tracing
 *
 * EndPoint = RK4(StartPoint, GradientVolume, StepSize);
 *
 * inputs :
 *      StartPoint: 2D or 3D location in vectorfield
 *      GradientVolume: Vectorfield
 *      Stepsize : The stepsize
 *
 * outputs :
 *      EndPoint : The new location (zero if outside image)
 *
 * Function is written by D.Kroon University of Twente (July 2008)
 */

    // MIndex2 is already defined in common.c
 //int mindex2(int x, int y, int sizx)  { return y*sizx+x;}


 int checkBounds2d( double *point, int *Isize) {
    if((point[0]<0)||(point[1]<0)||(point[0]>(Isize[0]-1))||(point[1]>(Isize[1]-1))) { return false; }
    return true;
}


 double norm2(double *a) { return sqrt(a[0]*a[0]+a[1]*a[1]); }



 void interpgrad2d(double *Ireturn, double *I, int *Isize, double *point) {
    /*  Linear interpolation variables */
    int xBas0, xBas1, yBas0, yBas1;
    double perc[4]={0, 0, 0, 0};
    double xCom, yCom, xComi, yComi;
    double fTlocalx, fTlocaly;
    int f;
    int index[4];
    
    fTlocalx = floor(point[0]); fTlocaly = floor(point[1]);
    xBas0=(int) fTlocalx; yBas0=(int) fTlocaly;
    xBas1=xBas0+1; yBas1=yBas0+1;
    
    /* Linear interpolation constants (percentages) */
    xCom=point[0]-fTlocalx; 
    yCom=point[1]-fTlocaly;
    xComi=(1-xCom); 
    yComi=(1-yCom);
    perc[0]=xComi * yComi;
    perc[1]=xComi * yCom;
    perc[2]=xCom * yComi;
    perc[3]=xCom * yCom;
    
    /* Stick to boundary */
    if(xBas0<0) { xBas0=0; if(xBas1<0) { xBas1=0; }}
    if(yBas0<0) { yBas0=0; if(yBas1<0) { yBas1=0; }}
    if(xBas1>(Isize[0]-1)) { xBas1=Isize[0]-1; if(xBas0>(Isize[0]-1)) { xBas0=Isize[0]-1; }}
    if(yBas1>(Isize[1]-1)) { yBas1=Isize[1]-1; if(yBas0>(Isize[1]-1)) { yBas0=Isize[1]-1; }}
    
    /* Get the neighbour intensities */
    index[0]=mindex2(xBas0, yBas0, Isize[0]);
    index[1]=mindex2(xBas0, yBas1, Isize[0]);
    index[2]=mindex2(xBas1, yBas0, Isize[0]);
    index[3]=mindex2(xBas1, yBas1, Isize[0]);
    f=Isize[0]*Isize[1];

    // Print debug information
    printf("\nInterpolation at point (%.2f, %.2f):\n", point[0], point[1]);
    printf("Base coordinates: x0=%d, x1=%d, y0=%d, y1=%d\n", xBas0, xBas1, yBas0, yBas1);
    printf("Indices for corners:\n");
    printf("Top-left     [%d,%d]: index=%d, dx=%.4f, dy=%.4f, weight=%.4f\n", 
           xBas0, yBas0, index[0], I[index[0]], I[index[0]+f], perc[0]);
    printf("Bottom-left  [%d,%d]: index=%d, dx=%.4f, dy=%.4f, weight=%.4f\n", 
           xBas0, yBas1, index[1], I[index[1]], I[index[1]+f], perc[1]);
    printf("Top-right    [%d,%d]: index=%d, dx=%.4f, dy=%.4f, weight=%.4f\n", 
           xBas1, yBas0, index[2], I[index[2]], I[index[2]+f], perc[2]);
    printf("Bottom-right [%d,%d]: index=%d, dx=%.4f, dy=%.4f, weight=%.4f\n", 
           xBas1, yBas1, index[3], I[index[3]], I[index[3]+f], perc[3]);
    
    /* the interpolated color */
    Ireturn[0]=I[index[0]]*perc[0]+I[index[1]]*perc[1]+I[index[2]]*perc[2]+I[index[3]]*perc[3];
    Ireturn[1]=I[index[0]+f]*perc[0]+I[index[1]+f]*perc[1]+I[index[2]+f]*perc[2]+I[index[3]+f]*perc[3];
}

 
bool RK4STEP_2D(double *gradientArray, int *gradientArraySize, double *startPoint, double *nextPoint, double stepSize) {
    /* Perform one step of the RK4 algorithm */
    double k1[2], k2[2], k3[2], k4[2];
    double tempPoint[2];
    double tempnorm;
    /*double D[2],dl;*/
    
   /*Calculate k1 */
    interpgrad2d(k1, gradientArray, gradientArraySize, startPoint);
    tempnorm=norm2(k1);
    k1[0] = k1[0]*stepSize/tempnorm;
    k1[1] = k1[1]*stepSize/tempnorm;
    
    tempPoint[0]=startPoint[0] + k1[0]*0.5;
    tempPoint[1]=startPoint[1] + k1[1]*0.5;
    
   /*Check the if are still inside the domain */
    if (!checkBounds2d(tempPoint, gradientArraySize)) return false;
    
   /*Calculate k2 */
    interpgrad2d(k2, gradientArray, gradientArraySize, tempPoint);
    tempnorm=norm2(k2);
    k2[0] = k2[0]*stepSize/tempnorm;
    k2[1] = k2[1]*stepSize/tempnorm;
    
    tempPoint[0]=startPoint[0] + k2[0]*0.5;
    tempPoint[1]=startPoint[1] + k2[1]*0.5;
    
   /*Check the if are still inside the domain */
    if (!checkBounds2d(tempPoint, gradientArraySize)) return false;
    
   /*Calculate k3 */
    interpgrad2d(k3, gradientArray, gradientArraySize, tempPoint);
    tempnorm=norm2(k3);
    k3[0] = k3[0]*stepSize/tempnorm;
    k3[1] = k3[1]*stepSize/tempnorm;
    
    tempPoint[0]=startPoint[0] + k3[0];
    tempPoint[1]=startPoint[1] + k3[1];
    
   /*Check the if are still inside the domain */
    if (!checkBounds2d(tempPoint, gradientArraySize)) return false;
    
   /*Calculate k4 */
    interpgrad2d(k4, gradientArray, gradientArraySize, tempPoint);
    tempnorm=norm2(k4);
    k4[0] = k4[0]*stepSize/tempnorm;
    k4[1] = k4[1]*stepSize/tempnorm;
    
     // Print final k-values
     printf("\nFinal RK4 coefficients:\n");
     printf("k1: (%.4f, %.4f) - scaled: (%.4f, %.4f)\n", 
            k1[0]/stepSize*tempnorm, k1[1]/stepSize*tempnorm, k1[0], k1[1]);
     printf("k2: (%.4f, %.4f) - scaled: (%.4f, %.4f)\n", 
            k2[0]/stepSize*tempnorm, k2[1]/stepSize*tempnorm, k2[0], k2[1]);
     printf("k3: (%.4f, %.4f) - scaled: (%.4f, %.4f)\n", 
            k3[0]/stepSize*tempnorm, k3[1]/stepSize*tempnorm, k3[0], k3[1]);
     printf("k4: (%.4f, %.4f) - scaled: (%.4f, %.4f)\n", 
            k4[0]/stepSize*tempnorm, k4[1]/stepSize*tempnorm, k4[0], k4[1]);
     printf("Step size used: %.4f\n", stepSize);
     printf("Movement vector: (%.4f, %.4f)\n", nextPoint[0] - startPoint[0], nextPoint[1] - startPoint[1]);

   /*Calculate final point */
    nextPoint[0] = startPoint[0] + (k1[0] + k2[0]*2.0 + k3[0]*2.0 + k4[0])/6.0;
    nextPoint[1] = startPoint[1] + (k1[1] + k2[1]*2.0 + k3[1]*2.0 + k4[1])/6.0;
    
    /* Set step to step size */
    /*
    D[0]=(nextPoint[0]-startPoint[0]);
    D[1]=(nextPoint[1]-startPoint[1]);
    dl=stepSize/(sqrt(D[0]*D[0]+D[1]*D[1])+1e-15);
    D[0]*=dl; D[1]*=dl;
    nextPoint[0]=startPoint[0]+D[0];
    nextPoint[1]=startPoint[1]+D[1];
    */
    
   /*Check the if are still inside the domain */
    if (!checkBounds2d(nextPoint, gradientArraySize)) return false;
    
    return true;
}

//solo funciona con un punto inicial
void gradient_descend_rk4(double* point, double *matriz, int filas, int columnas, double *new_point, double step) {
    /*
    double *gradientArray;
    const mwSize *gradientArraySizeC;
    const mwSize *PointSizeC;
    mwSize gradientArraySize[3];
    mwSize PointDims;
    mwSize gradientDims;
    double *startPoint;
    */

    double *nextPoint = new_point;
    int stepSizeArray;
    double stepSize = step;
    int i;

    double startPoint1[2];
    int PointLength=1;
    int gradientArraySize[2];
    int PointSizeC[2];
    int gradientArraySizeC[2];

    int size_map[2] = {columnas,filas};
    int size_point[2] = {2,1};
    gradientArraySizeC[0] = size_map[0];
    gradientArraySizeC[1] = size_map[1];
    PointSizeC[0] = size_point[0];
    PointSizeC[1] = size_point[1];

    /*  Get the number of gradient dimensions 
    gradientDims=mxGetNumberOfDimensions(prhs[1]);
    
   Get the size of the gradient Array 
    gradientArraySizeC = mxGetDimensions(prhs[1]);
    
     Get the number of startingpoint dimensions 
    PointDims=mxGetNumberOfDimensions(prhs[0]);
    
   Get the size of the startingpoint 
    PointSizeC = mxGetDimensions(prhs[0]);*/
    
    for (i=0; i<PointSizeC[1]; i++) { PointLength=PointLength*PointSizeC[i]; }
    
    
   /*Connect inputs */
    double* startPoint = point;
    double* gradientArray = matriz;
    //startPoint =  mxGetPr(prhs[0]);
    //gradientArray = mxGetPr(prhs[1]);
    //stepSizeArray = mxGetPr(prhs[2]); stepSize=stepSizeArray[0];
    //plhs[0] = mxCreateNumericArray(2, PointSizeC, mxDOUBLE_CLASS, mxREAL);
    //nextPoint= mxGetPr(plhs[0]);
    
   /*Perform the RK4 raytracing step */
    if(PointLength==2) {
        gradientArraySize[0]=gradientArraySizeC[0];
        gradientArraySize[1]=gradientArraySizeC[1];
        startPoint1[0]=startPoint[0]-1.0; 
        startPoint1[1]=startPoint[1]-1.0;
        if(RK4STEP_2D(gradientArray, gradientArraySize, startPoint1, nextPoint, stepSize)) {
            nextPoint[0]=nextPoint[0]+1.0; 
            nextPoint[1]=nextPoint[1]+1.0;
            printf("\nGradient descent step:\n");
            printf("Start point: (%.2f, %.2f)\n", startPoint[0], startPoint[1]);
            printf("Next point:  (%.2f, %.2f)\n", nextPoint[0], nextPoint[1]);
            printf("Step size: %d\n", stepSize);
        }
    
        else {
            nextPoint[0]=0; nextPoint[1]=0;
        }
    }
    else {
        printf("Starting Point must be 2D or 3D");
    }
}

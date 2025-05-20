#include "math.h"
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>

#include "../common.h"
#include "rk4_2D_3D.h"

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


 int checkBounds2d( float *point, int *Isize) {
    if((point[0]<0)||(point[1]<0)||(point[0]>(Isize[0]-1))||(point[1]>(Isize[1]-1))) { return false; }
    return true;
}

int checkBounds3d( float *point, int *Isize) {
    if((point[0]<0)||(point[1]<0)||(point[2]<0)||(point[0]>(Isize[0]-1))||(point[1]>(Isize[1]-1))||(point[2]>(Isize[2]-1))) { return false; }
    return true;
}


 float norm2(float *a) { return sqrt(a[0]*a[0]+a[1]*a[1]); }
 float norm3(float *a) { return sqrt(a[0]*a[0]+a[1]*a[1]+a[2]*a[2]); }


 void interpgrad2d(float *Ireturn, float *I, int *Isize, float *point) {
    /*  Linear interpolation variables */
    int xBas0, xBas1, yBas0, yBas1;
    float perc[4]={0, 0, 0, 0};
    float xCom, yCom, xComi, yComi;
    float fTlocalx, fTlocaly;
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
    
    /* the interpolated color */
    Ireturn[0]=I[index[0]]*perc[0]+I[index[1]]*perc[1]+I[index[2]]*perc[2]+I[index[3]]*perc[3];
    Ireturn[1]=I[index[0]+f]*perc[0]+I[index[1]+f]*perc[1]+I[index[2]+f]*perc[2]+I[index[3]+f]*perc[3];
}

void interpgrad3d(float *Ireturn, float *I, int *Isize, float *point) {
    /*  Linear interpolation variables */
    int xBas0, xBas1, yBas0, yBas1, zBas0, zBas1;
    float perc[8];
    float xCom, yCom, zCom;
    float xComi, yComi, zComi;
    float fTlocalx, fTlocaly, fTlocalz;
    int f0, f1;
    int index[8];
    float temp;
    
    fTlocalx = floor(point[0]); fTlocaly = floor(point[1]); fTlocalz = floor(point[2]);
    xBas0=(int) fTlocalx; yBas0=(int) fTlocaly; zBas0=(int) fTlocalz;
    xBas1=xBas0+1; yBas1=yBas0+1; zBas1=zBas0+1;
    
    /* Linear interpolation constants (percentages) */
    xCom=point[0]-fTlocalx;  yCom=point[1]-fTlocaly;   zCom=point[2]-fTlocalz;
    xComi=(1-xCom); yComi=(1-yCom); zComi=(1-zCom);
    perc[0]=xComi * yComi; perc[1]=perc[0] * zCom; perc[0]=perc[0] * zComi;
    perc[2]=xComi * yCom;  perc[3]=perc[2] * zCom; perc[2]=perc[2] * zComi;
    perc[4]=xCom * yComi;  perc[5]=perc[4] * zCom; perc[4]=perc[4] * zComi;
    perc[6]=xCom * yCom;   perc[7]=perc[6] * zCom; perc[6]=perc[6] * zComi;
    
    /* Stick to boundary */
    if(xBas0<0) { xBas0=0; if(xBas1<0) { xBas1=0; }}
    if(yBas0<0) { yBas0=0; if(yBas1<0) { yBas1=0; }}
    if(zBas0<0) { zBas0=0; if(zBas1<0) { zBas1=0; }}

    if(xBas1>(Isize[0]-1)) { xBas1=Isize[0]-1; if(xBas0>(Isize[0]-1)) { xBas0=Isize[0]-1; }}
    if(yBas1>(Isize[1]-1)) { yBas1=Isize[1]-1; if(yBas0>(Isize[1]-1)) { yBas0=Isize[1]-1; }}
    if(zBas1>(Isize[2]-1)) { zBas1=Isize[2]-1; if(zBas0>(Isize[2]-1)) { zBas0=Isize[2]-1; }}
    
    
   /*Get the neighbour intensities */
    index[0]=mindex3(xBas0, yBas0, zBas0, Isize[0], Isize[1]);
    index[1]=mindex3(xBas0, yBas0, zBas1, Isize[0], Isize[1]);
    index[2]=mindex3(xBas0, yBas1, zBas0, Isize[0], Isize[1]);
    index[3]=mindex3(xBas0, yBas1, zBas1, Isize[0], Isize[1]);
    index[4]=mindex3(xBas1, yBas0, zBas0, Isize[0], Isize[1]);
    index[5]=mindex3(xBas1, yBas0, zBas1, Isize[0], Isize[1]);
    index[6]=mindex3(xBas1, yBas1, zBas0, Isize[0], Isize[1]);
    index[7]=mindex3(xBas1, yBas1, zBas1, Isize[0], Isize[1]);
    f0=Isize[0]*Isize[1]*Isize[2];
    f1=f0+f0;
    
   /*the interpolated color */
    temp=I[index[0]]*perc[0]+I[index[1]]*perc[1]+I[index[2]]*perc[2]+I[index[3]]*perc[3];
    Ireturn[0]=temp+I[index[4]]*perc[4]+I[index[5]]*perc[5]+I[index[6]]*perc[6]+I[index[7]]*perc[7];
    temp=I[index[0]+f0]*perc[0]+I[index[1]+f0]*perc[1]+I[index[2]+f0]*perc[2]+I[index[3]+f0]*perc[3];
    Ireturn[1]=temp+I[index[4]+f0]*perc[4]+I[index[5]+f0]*perc[5]+I[index[6]+f0]*perc[6]+I[index[7]+f0]*perc[7];
    temp=I[index[0]+f1]*perc[0]+I[index[1]+f1]*perc[1]+I[index[2]+f1]*perc[2]+I[index[3]+f1]*perc[3];
    Ireturn[2]=temp+I[index[4]+f1]*perc[4]+I[index[5]+f1]*perc[5]+I[index[6]+f1]*perc[6]+I[index[7]+f1]*perc[7];
}
 
bool RK4STEP_2D(float *gradientArray, int *gradientArraySize, float *startPoint, float *nextPoint, float stepSize) {
    /* Perform one step of the RK4 algorithm */
    float k1[2], k2[2], k3[2], k4[2];
    float tempPoint[2];
    float tempnorm;
    /*float D[2],dl;*/
    
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

bool RK4STEP_3D(float *gradientArray, int *gradientArraySize, float *startPoint, float *nextPoint, float stepSize) {
    float k1[3], k2[3], k3[3], k4[3];
    float tempPoint[3];
    float tempnorm;
    
   /*Calculate k1 */
    interpgrad3d(k1, gradientArray, gradientArraySize, startPoint);
    tempnorm=norm3(k1);
    k1[0] = k1[0]*stepSize/tempnorm;
    k1[1] = k1[1]*stepSize/tempnorm;
    k1[2] = k1[2]*stepSize/tempnorm;
    
    tempPoint[0]=startPoint[0] + k1[0]*0.5;
    tempPoint[1]=startPoint[1] + k1[1]*0.5;
    tempPoint[2]=startPoint[2] + k1[2]*0.5;
            
   /*Check the if are still inside the domain */
    if (!checkBounds3d(tempPoint, gradientArraySize)) return false;
    
   /*Calculate k2 */
    interpgrad3d(k2, gradientArray, gradientArraySize, tempPoint);
    tempnorm=norm3(k2);
    k2[0] = k2[0]*stepSize/tempnorm;
    k2[1] = k2[1]*stepSize/tempnorm;
    k2[2] = k2[2]*stepSize/tempnorm;
    
    tempPoint[0]=startPoint[0] + k2[0]*0.5;
    tempPoint[1]=startPoint[1] + k2[1]*0.5;
    tempPoint[2]=startPoint[2] + k2[2]*0.5;
    
   /*Check the if are still inside the domain */
    if (!checkBounds3d(tempPoint, gradientArraySize)) return false;
    
   /*Calculate k3 */
    interpgrad3d(k3, gradientArray, gradientArraySize, tempPoint);
    tempnorm=norm3(k3);
    k3[0] = k3[0]*stepSize/tempnorm;
    k3[1] = k3[1]*stepSize/tempnorm;
    k3[2] = k3[2]*stepSize/tempnorm;
        
    tempPoint[0]=startPoint[0] + k3[0];
    tempPoint[1]=startPoint[1] + k3[1];
    tempPoint[2]=startPoint[2] + k3[2];
    
   /*Check the if are still inside the domain */
    if (!checkBounds3d(tempPoint, gradientArraySize)) return false;
    
   /*Calculate k4 */
    interpgrad3d(k4, gradientArray, gradientArraySize, tempPoint);
    tempnorm=norm3(k4);
    k4[0] = k4[0]*stepSize/tempnorm;
    k4[1] = k4[1]*stepSize/tempnorm;
    k4[2] = k4[2]*stepSize/tempnorm;
    
   /*Calculate final point */
    nextPoint[0] = startPoint[0] + (k1[0] + k2[0]*2.0 + k3[0]*2.0 + k4[0])/6.0;
    nextPoint[1] = startPoint[1] + (k1[1] + k2[1]*2.0 + k3[1]*2.0 + k4[1])/6.0;
    nextPoint[2] = startPoint[2] + (k1[2] + k2[2]*2.0 + k3[2]*2.0 + k4[2])/6.0;
     
   /*Check the if are still inside the domain */
    if (!checkBounds3d(nextPoint, gradientArraySize)) return false;
    
    return true;
}
//solo funciona con un punto inicial
void gradient_descend_rk4(float* startPoint, float *gradientArray, int* size_map, int* size_point, float *nextPoint, float stepSize) {

    int PointLength;
    int PointSizeC[2];


    PointSizeC[0] = size_point[0];
    PointSizeC[1] = size_point[1];

    
   //Get the size of the startingpoint 

    
   PointLength = PointSizeC[0];
    
    
   /*Perform the RK4 raytracing step */
    if(PointLength==2) {
        int gradientArraySize[2];
        float startPoint1[2];
        gradientArraySize[0]=size_map[0];
        gradientArraySize[1]=size_map[1];
        startPoint1[0]=startPoint[0]-1.0; 
        startPoint1[1]=startPoint[1]-1.0;
        if(RK4STEP_2D(gradientArray, gradientArraySize, startPoint1, nextPoint, stepSize)) {
            nextPoint[0]=nextPoint[0]+1.0; 
            nextPoint[1]=nextPoint[1]+1.0;
        }
    
        else {
            nextPoint[0]=0; nextPoint[1]=0;
        }
    }
    else if(PointLength==3) {
        int gradientArraySize[3];
        float startPoint1[3];
        gradientArraySize[0]=size_map[0];
        gradientArraySize[1]=size_map[1];
        gradientArraySize[2]=size_map[2];

        startPoint1[0]=startPoint[0]-1.0; 
        startPoint1[1]=startPoint[1]-1.0; 
        startPoint1[2]=startPoint[2]-1.0;
        if(RK4STEP_3D(gradientArray, gradientArraySize, startPoint1, nextPoint, stepSize)) {
            nextPoint[0]=nextPoint[0]+1.0; 
            nextPoint[1]=nextPoint[1]+1.0; 
            nextPoint[2]=nextPoint[2]+1.0; 
        }
        else {
            nextPoint[0]=0; nextPoint[1]=0; nextPoint[2]=0;
        }
        
    }
    else {
        printf("Starting Point must be 2D or 3D");
    }
}

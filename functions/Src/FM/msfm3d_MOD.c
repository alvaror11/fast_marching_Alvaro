#include <stdbool.h>
#include <stdlib.h>
#include "math.h"
#include "../common.h"
#include "msfm3d_MOD.h"




/*This function MSFM3D calculates the shortest distance from a list of */
/*points to all other pixels in an image, using the */
/*Multistencil Fast Marching Method (MSFM). This method gives more accurate */
/*distances by using second order derivatives and cross neighbours. */
/* */
/*T=msfm3d(F, SourcePoints, UseSecond, UseCross) */
/* */
/*inputs, */
/*   F: The 3D speed image. The speed function must always be larger */
/*			than zero (min value 1e-8), otherwise some regions will */
/*			never be reached because the time will go to infinity.  */
/*  SourcePoints : A list of starting points [3 x N] (distance zero) */
/*  UseSecond : Boolean Set to true if not only first but also second */
/*               order derivatives are used (default) */
/*  UseCross: Boolean Set to true if also cross neighbours */
/*               are used (default) */
/*outputs, */
/*  T : Image with distance from SourcePoints to all pixels */

/* */
/*Function is written by D.Kroon University of Twente (June 2009) */



float second_derivative(float Txm1, float Txm2, float Txp1, float Txp2) {
    bool ch1, ch2;
    float Tm;
    Tm=INF;
    ch1=(Txm2<Txm1)&&IsFinite(Txm1); ch2=(Txp2<Txp1)&&IsFinite(Txp1);
    if(ch1&&ch2) { Tm =min( (4.0*Txm1-Txm2)/3.0 , (4.0*Txp1-Txp2)/3.0);}
    else if(ch1) { Tm =(4.0*Txm1-Txm2)/3.0; }
    else if(ch2) { Tm =(4.0*Txp1-Txp2)/3.0; }
    return Tm;
}

float CalculateDistance3D(float *T, float Fijk, int *dims, int i, int j, int k, bool usesecond, bool usecross, bool *Frozen) {
    
    /* Loop variables */
    int q, t;
    
    /* Current location */
    int in, jn, kn;
    
    /* Derivatives */
    float Tm[18]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float Tm2[18]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float Coeff[3];
    
    /* local derivatives in distance image */
    float Txm1, Txm2, Txp1, Txp2;
    float Tym1, Tym2, Typ1, Typ2;
    float Tzm1, Tzm2, Tzp1, Tzp2;
    /* local cross derivatives in distance image */
    float Tr2t1m1, Tr2t1m2, Tr2t1p1, Tr2t1p2;
    float Tr2t2m1, Tr2t2m2, Tr2t2p1, Tr2t2p2;
    float Tr2t3m1, Tr2t3m2, Tr2t3p1, Tr2t3p2;
    float Tr3t1m1, Tr3t1m2, Tr3t1p1, Tr3t1p2;
    float Tr3t2m1, Tr3t2m2, Tr3t2p1, Tr3t2p2;
    float Tr3t3m1, Tr3t3m2, Tr3t3p1, Tr3t3p2;
    float Tr4t1m1, Tr4t1m2, Tr4t1p1, Tr4t1p2;
    float Tr4t2m1, Tr4t2m2, Tr4t2p1, Tr4t2p2;
    float Tr4t3m1, Tr4t3m2, Tr4t3p1, Tr4t3p2;
    float Tr5t1m1, Tr5t1m2, Tr5t1p1, Tr5t1p2;
    float Tr5t2m1, Tr5t2m2, Tr5t2p1, Tr5t2p2;
    float Tr5t3m1, Tr5t3m2, Tr5t3p1, Tr5t3p2;
    float Tr6t1m1, Tr6t1m2, Tr6t1p1, Tr6t1p2;
    float Tr6t2m1, Tr6t2m2, Tr6t2p1, Tr6t2p2;
    float Tr6t3m1, Tr6t3m2, Tr6t3p1, Tr6t3p2;
    
    float Tt, Tt2;
    
    
    /* Return values root of polynomial */
    float ansroot[2]={0, 0};
    
    /* Order derivatives in a certain direction */
    int Order[18]={0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    
    /* Stencil constants */
    float G1[18]={1, 1, 1, 1, 0.5, 0.5, 1, 0.5, 0.5, 1, 0.5, 0.5, 0.5, 0.3333333333333, 0.3333333333333, 0.5, 0.3333333333333, 0.3333333333333};
    float G2[18]={2.250, 2.250, 2.250, 2.250, 1.125, 1.125, 2.250, 1.125, 1.125, 2.250, 1.125, 1.125, 1.125, 0.750, 0.750, 1.125, 0.750, 0.750};
    
    
    /*Get First order derivatives (only use frozen pixel) */
    in=i-1; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txm1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txm1=INF; }
    in=i+1; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txp1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txp1=INF; }
    in=i+0; jn=j-1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tym1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tym1=INF; }
    in=i+0; jn=j+1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Typ1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Typ1=INF; }
    in=i+0; jn=j+0; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzm1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzm1=INF; }
    in=i+0; jn=j+0; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzp1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzp1=INF; }
    
    if(usecross) {
        Tr2t1m1=Txm1;
        Tr2t1p1=Txp1;
        in=i-0; jn=j-1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2m1=INF; }
        in=i+0; jn=j+1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2p1=INF; }
        in=i-0; jn=j-1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3m1=INF; }
        in=i+0; jn=j+1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3p1=INF; }
        Tr3t1m1=Tym1;
        Tr3t1p1=Typ1;
        in=i-1; jn=j+0; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2m1=INF; }
        in=i+1; jn=j+0; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2p1=INF; }
        in=i-1; jn=j-0; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3m1=INF; }
        in=i+1; jn=j+0; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3p1=INF; }
        Tr4t1m1=Tzm1;
        Tr4t1p1=Tzp1;
        in=i-1; jn=j-1; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2m1=INF; }
        in=i+1; jn=j+1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2p1=INF; }
        in=i-1; jn=j+1; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3m1=INF; }
        in=i+1; jn=j-1; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3p1=INF; }
        Tr5t1m1=Tr3t3m1;
        Tr5t1p1=Tr3t3p1;
        in=i-1; jn=j-1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2m1=INF; }
        in=i+1; jn=j+1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2p1=INF; }
        in=i-1; jn=j+1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3m1=INF; }
        in=i+1; jn=j-1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3p1=INF; }
        Tr6t1m1=Tr3t2p1;
        Tr6t1p1=Tr3t2m1;
        in=i-1; jn=j-1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2m1=INF; }
        in=i+1; jn=j+1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2p1=INF; }
        in=i-1; jn=j+1; kn=k-1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3m1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3m1=INF; }
        in=i+1; jn=j-1; kn=k+1; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3p1=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3p1=INF; }
    }
    
    /*The values in order is 0 if no neighbours in that direction */
    /*1 if 1e order derivatives is used and 2 if second order */
    /*derivatives are used */
    
    
    /*Make 1e order derivatives in x and y direction */
    Tm[0] = min( Txm1 , Txp1); if(IsFinite(Tm[0])){ Order[0]=1; } else { Order[0]=0; }
    Tm[1] = min( Tym1 , Typ1); if(IsFinite(Tm[1])){ Order[1]=1; } else { Order[1]=0; }
    Tm[2] = min( Tzm1 , Tzp1); if(IsFinite(Tm[2])){ Order[2]=1; } else { Order[2]=0; }
    
    /*Make 1e order derivatives in cross directions */
    if(usecross) {
        Tm[3] = Tm[0]; Order[3]=Order[0];
        Tm[4] = min( Tr2t2m1 , Tr2t2p1); if(IsFinite(Tm[4])){ Order[4]=1; } else { Order[4]=0; }
        Tm[5] = min( Tr2t3m1 , Tr2t3p1); if(IsFinite(Tm[5])){ Order[5]=1; } else { Order[5]=0; }
        
        Tm[6] = Tm[1]; Order[6]=Order[1];
        Tm[7] = min( Tr3t2m1 , Tr3t2p1); if(IsFinite(Tm[7])){ Order[7]=1; } else { Order[7]=0; }
        Tm[8] = min( Tr3t3m1 , Tr3t3p1); if(IsFinite(Tm[8])){ Order[8]=1; } else { Order[8]=0; }
        
        Tm[9] = Tm[2]; Order[9]=Order[2];
        Tm[10] = min( Tr4t2m1 , Tr4t2p1); if(IsFinite(Tm[10])){ Order[10]=1; } else { Order[10]=0; }
        Tm[11] = min( Tr4t3m1 , Tr4t3p1); if(IsFinite(Tm[11])){ Order[11]=1; } else { Order[11]=0; }
        
        Tm[12] = Tm[8]; Order[12]=Order[8];
        Tm[13] = min( Tr5t2m1 , Tr5t2p1); if(IsFinite(Tm[13])){ Order[13]=1; } else { Order[13]=0; }
        Tm[14] = min( Tr5t3m1 , Tr5t3p1); if(IsFinite(Tm[14])){ Order[14]=1; } else { Order[14]=0; }
        
        Tm[15] = Tm[7]; Order[15]=Order[7];
        Tm[16] = min( Tr6t2m1 , Tr6t2p1); if(IsFinite(Tm[16])){ Order[16]=1; } else { Order[16]=0; }
        Tm[17] = min( Tr6t3m1 , Tr6t3p1); if(IsFinite(Tm[17])){ Order[17]=1; } else { Order[17]=0; }
    }
    
    /*Make 2e order derivatives */
    if(usesecond) {
        /*Get Second order derivatives (only use frozen pixel) */
        /*Get First order derivatives (only use frozen pixel) */
        in=i-2; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txm2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txm2=INF; }
        in=i+2; jn=j+0; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Txp2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Txp2=INF; }
        in=i+0; jn=j-2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tym2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tym2=INF; }
        in=i+0; jn=j+2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Typ2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Typ2=INF; }
        in=i+0; jn=j+0; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzm2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzm2=INF; }
        in=i+0; jn=j+0; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tzp2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tzp2=INF; }
        
        if(usecross) {
            Tr2t1m2=Txm2;
            Tr2t1p2=Txp2;
            in=i-0; jn=j-2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2m2=INF; }
            in=i+0; jn=j+2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t2p2=INF; }
            in=i-0; jn=j-2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3m2=INF; }
            in=i+0; jn=j+2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr2t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr2t3p2=INF; }
            Tr3t1m2=Tym2;
            Tr3t1p2=Typ2;
            in=i-2; jn=j+0; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2m2=INF; }
            in=i+2; jn=j+0; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t2p2=INF; }
            in=i-2; jn=j-0; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3m2=INF; }
            in=i+2; jn=j+0; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr3t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr3t3p2=INF; }
            Tr4t1m2=Tzm2;
            Tr4t1p2=Tzp2;
            in=i-2; jn=j-2; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2m2=INF; }
            in=i+2; jn=j+2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t2p2=INF; }
            in=i-2; jn=j+2; kn=k-0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3m2=INF; }
            in=i+2; jn=j-2; kn=k+0; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr4t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr4t3p2=INF; }
            Tr5t1m2=Tr3t3m2;
            Tr5t1p2=Tr3t3p2;
            in=i-2; jn=j-2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2m2=INF; }
            in=i+2; jn=j+2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t2p2=INF; }
            in=i-2; jn=j+2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3m2=INF; }
            in=i+2; jn=j-2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr5t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr5t3p2=INF; }
            Tr6t1m2=Tr3t2p2;
            Tr6t1p2=Tr3t2m2;
            in=i-2; jn=j-2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2m2=INF; }
            in=i+2; jn=j+2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t2p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t2p2=INF; }
            in=i-2; jn=j+2; kn=k-2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3m2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3m2=INF; }
            in=i+2; jn=j-2; kn=k+2; if(isfrozen3d(in, jn, kn, dims, Frozen)) { Tr6t3p2=T[mindex3(in, jn, kn, dims[0], dims[1])]; } else { Tr6t3p2=INF; }
        }
        
        
        /*pixels with a pixeldistance 2 from the center must be */
        /*lower in value otherwise use other side or first order */
        
        Tm2[0]=second_derivative(Txm1, Txm2, Txp1, Txp2); if(IsInf(Tm2[0])) { Tm2[0]=0; } else { Order[0]=2; }
        Tm2[1]=second_derivative(Tym1, Tym2, Typ1, Typ2); if(IsInf(Tm2[1])) { Tm2[1]=0; } else { Order[1]=2; }
        Tm2[2]=second_derivative(Tzm1, Tzm2, Tzp1, Tzp2); if(IsInf(Tm2[2])) { Tm2[2]=0; } else { Order[2]=2; }
        
        if(usecross) {
            Tm2[3]=Tm2[0]; Order[3]=Order[0];
            Tm2[4]=second_derivative(Tr2t2m1, Tr2t2m2, Tr2t2p1, Tr2t2p2); if(IsInf(Tm2[4])) { Tm2[4]=0; } else { Order[4]=2; }
            Tm2[5]=second_derivative(Tr2t3m1, Tr2t3m2, Tr2t3p1, Tr2t3p2); if(IsInf(Tm2[5])) { Tm2[5]=0; } else { Order[5]=2; }
            
            Tm2[6]=Tm2[1]; Order[6]=Order[1];
            Tm2[7]=second_derivative(Tr3t2m1, Tr3t2m2, Tr3t2p1, Tr3t2p2); if(IsInf(Tm2[7])) { Tm2[7]=0; } else { Order[7]=2; }
            Tm2[8]=second_derivative(Tr3t3m1, Tr3t3m2, Tr3t3p1, Tr3t3p2); if(IsInf(Tm2[8])) { Tm2[8]=0; } else { Order[8]=2; }
            
            Tm2[9]=Tm2[2]; Order[9]=Order[2];
            Tm2[10]=second_derivative(Tr4t2m1, Tr4t2m2, Tr4t2p1, Tr4t2p2); if(IsInf(Tm2[10])) { Tm2[10]=0; } else { Order[10]=2; }
            Tm2[11]=second_derivative(Tr4t3m1, Tr4t3m2, Tr4t3p1, Tr4t3p2); if(IsInf(Tm2[11])) { Tm2[11]=0; } else { Order[11]=2; }
            
            Tm2[12]=Tm2[8]; Order[12]=Order[8];
            Tm2[13]=second_derivative(Tr5t2m1, Tr5t2m2, Tr5t2p1, Tr5t2p2); if(IsInf(Tm2[13])) { Tm2[13]=0; } else { Order[13]=2; }
            Tm2[14]=second_derivative(Tr5t3m1, Tr5t3m2, Tr5t3p1, Tr5t3p2); if(IsInf(Tm2[14])) { Tm2[14]=0; } else { Order[14]=2; }
            
            Tm2[15]=Tm2[7]; Order[15]=Order[7];
            Tm2[16]=second_derivative(Tr6t2m1, Tr6t2m2, Tr6t2p1, Tr6t2p2); if(IsInf(Tm2[16])) { Tm2[16]=0; } else { Order[16]=2; }
            Tm2[17]=second_derivative(Tr6t3m1, Tr6t3m2, Tr6t3p1, Tr6t3p2); if(IsInf(Tm2[17])) { Tm2[17]=0; } else { Order[17]=2; }
        }
        
    }
    
    /*Calculate the distance using x and y direction */
    Coeff[0]=0; Coeff[1]=0; Coeff[2]=-1/(max(pow2(Fijk),eps));
    
    for (t=0; t<3; t++) {
        switch(Order[t]) {
            case 1:
                Coeff[0]+=G1[t]; Coeff[1]+=-2.0*Tm[t]*G1[t]; Coeff[2]+=pow2(Tm[t])*G1[t];
                break;
            case 2:
                Coeff[0]+=G2[t]; Coeff[1]+=-2.0*Tm2[t]*G2[t]; Coeff[2]+=pow2(Tm2[t])*G2[t];
                break;
        }
    }
    
    
    roots(Coeff, ansroot);
    Tt=max(ansroot[0], ansroot[1]);
    
    /*Calculate the distance using the cross directions */
    if(usecross) {
        
        for(q=1; q<6; q++) {
            /* Original Equation */
            /*    Coeff[0]=0; Coeff[1]=0; Coeff[2]=-1/(max(pow2(Fijk),eps)) */
            Coeff[0]+=0; Coeff[1]+=0; Coeff[2]+=-1/(max(pow2(Fijk),eps));
            
            for (t=q*3; t<((q+1)*3); t++) {
                switch(Order[t]) {
                    case 1:
                        Coeff[0]+=G1[t]; Coeff[1]+=-2.0*Tm[t]*G1[t]; Coeff[2]+=pow2(Tm[t])*G1[t];
                        break;
                    case 2:
                        Coeff[0]+=G2[t]; Coeff[1]+=-2.0*Tm2[t]*G2[t]; Coeff[2]+=pow2(Tm2[t])*G2[t];
                        break;
                }
            }
            /*Select maximum root solution and minimum distance value of both stensils */
            if(Coeff[0]>0) { roots(Coeff, ansroot); Tt2=max(ansroot[0], ansroot[1]); Tt=min(Tt, Tt2); }
        }
    }
    
    /*Upwind condition check, current distance must be larger */
    /*then direct neighbours used in solution */
    /*(Will this ever happen?) */
    if(usecross) {
        for(q=0; q<18; q++) { if(IsFinite(Tm[q])&&(Tt<Tm[q])) { Tt=Tm[minarray(Tm, 18)]+(1/(max(Fijk,eps)));}}
    }
    else {
        for(q=0; q<3; q++) { if(IsFinite(Tm[q])&&(Tt<Tm[q])) { Tt=Tm[minarray(Tm, 3)]+(1/(max(Fijk,eps)));}}
    }
    
    return Tt;
}

/* The matlab mex function */
float* main_msfm3D(float* F, float* SourcePoints, float* T, int* size_map, int* size_target, float* start_points) {
    /* The input variables */
    bool usesecond=true;
    bool usecross=true;
    
    //printf("Starting MSFM3D\n");
    /* Euclidian distance image */
    float *Y;
    
    /* Current distance values */
    float Tt, Ty;
    
    /* Matrix containing the Frozen Pixels" */
    bool *Frozen;
    
    /* Augmented Fast Marching (For skeletonize) */
    bool Ed;
    
    /* Size of input image */
    int dims[3];
    
    /* Size of  SourcePoints array */
    size_t dims_sp[3];
    
    /* Number of pixels in image */
    int npixels;
    
    /* Neighbour list */
    int neg_free;
    int neg_pos;
    float *neg_listv;
    float *neg_listx;
    float *neg_listy;
    float *neg_listz;
    float *neg_listo;
    
    int *listprop;
    float **listval;
    
    /* Neighbours 6x3 */
    int ne[18]={-1,  0,  0, 1, 0, 0, 0, -1,  0, 0, 1, 0, 0,  0, -1, 0, 0, 1};
    
    /* Loop variables */
    int s, w, itt, q;
    
    /* Current location */
    int x, y, z, i, j, k;
    
    /* Index */
    int IJK_index, XYZ_index, index;
    
    
    /* Get the sizes of the input image volume and the source points */
    dims[0] = size_map[0];
    dims[1] = size_map[1];
    dims[2] = size_map[2];
    dims_sp[0] = size_target[0];
    dims_sp[1] = size_target[1];
    
    npixels=size_map[0]*size_map[1]*size_map[2];

    // Allocate memory for outputs. IT WONT BE NEEDED, allocate in prueba
    T = (float *)malloc(npixels * sizeof(float));
    if(Ed) { 
        Y = (float *)malloc(npixels * sizeof(float));
    }
    
    /* Pixels which are processed and have a final distance are frozen */
    Frozen = (bool*)malloc( npixels* sizeof(bool) );
    for(q=0;q<npixels;q++){Frozen[q]=0; T[q]=-1;}
    if(Ed) {
        for(q=0;q<npixels;q++){Y[q]=-1;}
    }
    
    
    /*Free memory to store neighbours of the (segmented) region */
    neg_free = 100000;
    neg_pos=0;
    
    neg_listx = (float *)malloc( neg_free*sizeof(float) );
    neg_listy = (float *)malloc( neg_free*sizeof(float) );
    neg_listz = (float *)malloc( neg_free*sizeof(float) );
    if(Ed) {
        neg_listo = (float *)malloc( neg_free*sizeof(float) );
        for(q=0;q<neg_free;q++) { neg_listo[q]=0; }
    }
    
    /* List parameters array */
    listprop=(int*)malloc(3* sizeof(int));
    /* Make jagged list to store a maximum of 2^64 values */
    listval= (float **)malloc( 64* sizeof(float *) );
    
    /* Initialize parameter list */
    initialize_list(listval, listprop);
    neg_listv=listval[listprop[1]-1];
    
    /*(There are 3 pixel classes: */
    /*  - frozen (processed) */
    /*  - narrow band (boundary) (in list to check for the next pixel with smallest distance) */
    /*  - far (not yet used) */
    /* set all starting points to distance zero and frozen */
    /* and add all neighbours of the starting points to narrow list */
    
    for (s=0; s<dims_sp[1]; s++) {
        /*starting point */
        x= (int)SourcePoints[0+s*3]-1;
        y= (int)SourcePoints[1+s*3]-1;
        z= (int)SourcePoints[2+s*3]-1;
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        
        Frozen[XYZ_index]=1;
        T[XYZ_index]=0;
        if(Ed) { Y[XYZ_index]=0; }
    }
    
    for (s=0; s<dims_sp[1]; s++) {
        /*starting point */
        x= (int)SourcePoints[0+s*3]-1;
        y= (int)SourcePoints[1+s*3]-1;
        
        z= (int)SourcePoints[2+s*3]-1;
        
        
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        for (w=0; w<6; w++) {
            /*Location of neighbour */
            i=x+ne[w];
            j=y+ne[w+6];
            k=z+ne[w+12];
            
            IJK_index=mindex3(i, j, k, dims[0], dims[1]);
            
            /*Check if current neighbour is not yet frozen and inside the */
            
            /*picture */
            if(isntfrozen3d(i, j, k, dims, Frozen)) {
                if (IJK_index < 0 || IJK_index >= (dims[0] * dims[1] * dims[2])) {
                    printf("Invalid array access detected:\n");
                    printf("Index: %d\n", IJK_index);
                    printf("Coordinates (i,j,k): (%d,%d,%d)\n", i, j, k);
                    printf("Dimensions: %d x %d x %d\n", dims[0], dims[1], dims[2]);
                    printf("Max valid index: %d\n", (dims[0] * dims[1] * dims[2]) - 1);
                    return NULL;
                }
    
                // Add F validation
                if (F == NULL) {
                    printf("Error: F is NULL\n");
                    return NULL;
                }
                
                Tt=(1/(max(F[IJK_index],eps)));
                /*Update distance in neigbour list or add to neigbour list */
                if(T[IJK_index]>0) {
                    if(neg_listv[(int)T[IJK_index]]>Tt) {
                        listupdate(listval, listprop, (int)T[IJK_index], Tt);
                    }
                }
                else {
                    /*If running out of memory at a new block */
                    if(neg_pos>=neg_free) {
                        neg_free+=100000;
                        neg_listx = (float *)realloc(neg_listx, neg_free*sizeof(float) );
                        neg_listy = (float *)realloc(neg_listy, neg_free*sizeof(float) );
                        neg_listz = (float *)realloc(neg_listz, neg_free*sizeof(float) );
                        if(Ed) {
                            neg_listo = (float *)realloc(neg_listo, neg_free*sizeof(float) );
                        }
                    }
                    list_add(listval, listprop, Tt);
                    neg_listv=listval[listprop[1]-1];
                    neg_listx[neg_pos]=i;
                    neg_listy[neg_pos]=j;
                    neg_listz[neg_pos]=k;
                    T[IJK_index]=neg_pos;
                    neg_pos++;
                }
            }
        }
    }
    
    /*Loop through all pixels of the image */
    for (itt=0; itt<(npixels); itt++) /* */ {
        /*Get the pixel from narrow list (boundary list) with smallest */
        /*distance value and set it to current pixel location */
        index=list_minimum(listval, listprop);
        neg_listv=listval[listprop[1]-1];
        /* Stop if pixel distance is infinite (all pixels are processed) */
        if(IsInf(neg_listv[index])) { break; }
        
        /*index=minarray(neg_listv, neg_pos); */
        x=(int)neg_listx[index]; y=(int)neg_listy[index]; z=(int)neg_listz[index];
        XYZ_index=mindex3(x, y, z, dims[0], dims[1]);
        Frozen[XYZ_index]=1;
        T[XYZ_index]=neg_listv[index];
        if(Ed) { Y[XYZ_index]=neg_listo[index]; }
        
        /*Remove min value by replacing it with the last value in the array */
        list_remove_replace(listval, listprop, index) ;
        neg_listv=listval[listprop[1]-1];
        if(index<(neg_pos-1)) {
            neg_listx[index]=neg_listx[neg_pos-1];
            neg_listy[index]=neg_listy[neg_pos-1];
            neg_listz[index]=neg_listz[neg_pos-1];
            if(Ed){
                neg_listo[index]=neg_listo[neg_pos-1];
            }
            T[(int)mindex3((int)neg_listx[index], (int)neg_listy[index], (int)neg_listz[index], dims[0], dims[1])]=index;
        }
        neg_pos =neg_pos-1;
        
        /*Loop through all 6 neighbours of current pixel */
        for (w=0;w<6;w++) {
            /*Location of neighbour */
            i=x+ne[w]; j=y+ne[w+6]; k=z+ne[w+12];
            IJK_index=mindex3(i, j, k, dims[0], dims[1]);
            
            /*Check if current neighbour is not yet frozen and inside the */
            /*picture */
            if(isntfrozen3d(i, j, k, dims, Frozen)) {
                Tt=CalculateDistance3D(T, F[IJK_index], dims, i, j, k, usesecond, usecross, Frozen);
                if(Ed) {
                    Ty=CalculateDistance3D(Y, 1, dims, i, j, k, usesecond, usecross, Frozen);
                }
                
                /*Update distance in neigbour list or add to neigbour list */
                IJK_index=mindex3(i, j, k, dims[0], dims[1]);
                if((T[IJK_index]>-1)&&T[IJK_index]<=listprop[0]) {
                    if(neg_listv[(int)T[IJK_index]]>Tt) {
                        listupdate(listval, listprop, (int)T[IJK_index], Tt);
                    }
                }
                else {
                    /*If running out of memory at a new block */
                    if(neg_pos>=neg_free) {
                        neg_free+=100000;
                        neg_listx = (float *)realloc(neg_listx, neg_free*sizeof(float) );
                        neg_listy = (float *)realloc(neg_listy, neg_free*sizeof(float) );
                        neg_listz = (float *)realloc(neg_listz, neg_free*sizeof(float) );
                        if(Ed) {
                            neg_listo = (float *)realloc(neg_listo, neg_free*sizeof(float) );
                        }
                    }
                    list_add(listval, listprop, Tt);
                    neg_listv=listval[listprop[1]-1];
                    neg_listx[neg_pos]=i; neg_listy[neg_pos]=j; neg_listz[neg_pos]=k;
                    if(Ed) {
                        neg_listo[neg_pos]=Ty;
                    }
                    
                    T[IJK_index]=neg_pos;
                    neg_pos++;
                    
                    if (x == start_points[0] && y == start_points[1] && z == start_points[2]) {
                        //stop the iteration of the loop if the starting point is reached
                        break;
                    }
                }
            }
        }
        
    }
    return (float*)T;
    /* Free memory */
    /* Destroy parameter list */
    destroy_list(listval, listprop);
    free(neg_listx);
    free(neg_listy);
    free(neg_listz);
    free(Frozen);
    free(Y);
}


void compute_gradient_3d_discrete(float* input_matrix, float* gradient_matrix, int* size_map){

    int ancho = size_map[0];
    int largo = size_map[1];
    int alto = size_map[2];
    int slice_size = ancho * largo;
    const int Ne[26][3] = {
        // Layer below (z-1)
        {-1, -1, -1}, {-1, 0, -1}, {-1, 1, -1},
        {0, -1, -1},  {0, 0, -1},  {0, 1, -1},
        {1, -1, -1},  {1, 0, -1},  {1, 1, -1},
        
        // Middle layer (z=0)
        {-1, -1, 0}, {-1, 0, 0}, {-1, 1, 0},
        {0, -1, 0},              {0, 1, 0},
        {1, -1, 0},  {1, 0, 0},  {1, 1, 0},
        
        // Layer above (z=1)
        {-1, -1, 1}, {-1, 0, 1}, {-1, 1, 1},
        {0, -1, 1},  {0, 0, 1},  {0, 1, 1},
        {1, -1, 1},  {1, 0, 1},  {1, 1, 1}
    };
    
    // For each point in the 3D matrix
    for(int k = 0; k < alto; k++) {
        for(int i = 0; i < ancho; i++) {
            for(int j = 0; j < largo; j++) {
                int current_idx = j + i*largo + k*slice_size;
                float current_value = input_matrix[current_idx];
                float min_value = current_value;
                float fx = 0, fy = 0, fz = 0;
                int valid_neighbors = 0;

                // Check all 26 neighbors
                for(int n = 0; n < 26; n++) {
                    int ni = i + Ne[n][0];
                    int nj = j + Ne[n][1];
                    int nk = k + Ne[n][2];
                   
                    // Only process neighbor if it's within bounds
                    if(ni >= 0 && ni < ancho && 
                       nj >= 0 && nj < largo && 
                       nk >= 0 && nk < alto) {
                        valid_neighbors++;
                        float neighbor_value = input_matrix[nj + ni*largo + nk*slice_size];

                        if(neighbor_value < min_value) {
                            min_value = neighbor_value;
                            // Normalize direction vector
                            float norm = sqrt(Ne[n][0]*Ne[n][0] + 
                                            Ne[n][1]*Ne[n][1] + 
                                            Ne[n][2]*Ne[n][2]);
                            fx = Ne[n][0]/norm; // x component
                            fy = Ne[n][1]/norm; // y component
                            fz = Ne[n][2]/norm; // z component
                        }
                    }
                }
                
                // Only store gradient if we found valid neighbors
                if(valid_neighbors > 0) {
                    gradient_matrix[current_idx] = fx;                    // x component
                    gradient_matrix[current_idx + slice_size*alto] = fy;  // y component
                    gradient_matrix[current_idx + 2*slice_size*alto] = fz; // z component

                } else {
                    printf("No valid neighbors found for point (%d, %d, %d)\n", i, j, k);
                    gradient_matrix[current_idx] = 0;
                    gradient_matrix[current_idx + slice_size*alto] = 0;
                    gradient_matrix[current_idx + 2*slice_size*alto] = 0;
                }
            }
        }
    }
}




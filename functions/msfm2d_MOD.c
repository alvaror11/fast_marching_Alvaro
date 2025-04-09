//#include "mex.h"
#include "math.h"
#include "common.h"
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "msfm2d_MOD.h"

/*
 * This function MSFM2D calculates the shortest distance from a list of
 * points to all other pixels in an image, using the
 *Multistencil Fast Marching Method (MSFM). This method gives more accurate
 *distances by using second order derivatives and cross neighbours.
 *
 *T=msfm2d(F, SourcePoints, UseSecond, UseCross)
 *
 *inputs,
 *  F: The speed image
 *  SourcePoints : A list of starting points [2 x N] (distance zero)
 *  UseSecond : Boolean Set to true if not only first but also second
 *               order derivatives are used (default)
 *  UseCross: Boolean Set to true if also cross neighbours
 *               are used (default)
 *outputs,
 *  T : Image with distance from SourcePoints to all pixels
 *
 *Function is written by D.Kroon University of Twente (June 2009)
 */

float CalculateDistance(float *T, float Fij, int *dims, int i, int j, bool usesecond, bool usecross, bool *Frozen) {
    /* Derivatives */
    float Tm[4]={0, 0, 0, 0};
    float Tm2[4]={0, 0, 0, 0};
    float Coeff[3];
    
    /* local derivatives in distance image */
    float Tpatch_2_3, Txm2, Tpatch_4_3, Txp2;
    float Tpatch_3_2, Tym2, Tpatch_3_4, Typ2;
    float Tpatch_2_2, Tr1m2, Tpatch_4_4, Tr1p2;
    float Tpatch_2_4, Tr2m2, Tpatch_4_2, Tr2p2;
    
    /* Return values root of polynomial */
    float ansroot[2]={0, 0};
    
    /* Loop variables  */
    int q, t;
    
    /* Derivative checks */
    bool ch1, ch2;
    
    /* Order derivatives in a certain direction */
    int Order[4]={0, 0, 0, 0};
    
    /* Current location */
    int in, jn;
    
    /* Constant cross term */
    const float c1=0.5;
    
    float Tt, Tt2;
    /*Get First order derivatives (only use frozen pixel)  */
    in=i-1; jn=j+0; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_2_3=T[mindex2(in, jn, dims[0])]; } else { Tpatch_2_3=INF; }
    in=i+0; jn=j-1; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_3_2=T[mindex2(in, jn, dims[0])]; } else { Tpatch_3_2=INF; }
    in=i+0; jn=j+1; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_3_4=T[mindex2(in, jn, dims[0])]; } else { Tpatch_3_4=INF; }
    in=i+1; jn=j+0; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_4_3=T[mindex2(in, jn, dims[0])]; } else { Tpatch_4_3=INF; }
    if(usecross) {
        in=i-1; jn=j-1; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_2_2=T[mindex2(in, jn, dims[0])]; } else { Tpatch_2_2=INF; }
        in=i-1; jn=j+1; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_2_4=T[mindex2(in, jn, dims[0])]; } else { Tpatch_2_4=INF; }
        in=i+1; jn=j-1; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_4_2=T[mindex2(in, jn, dims[0])]; } else { Tpatch_4_2=INF; }
        in=i+1; jn=j+1; if(isfrozen2d(in, jn, dims, Frozen)) { Tpatch_4_4=T[mindex2(in, jn, dims[0])]; } else { Tpatch_4_4=INF; }
    }
    /*The values in order is 0 if no neighbours in that direction  */
    /*1 if 1e order derivatives is used and 2 if second order  */
    /*derivatives are used  */
    Order[0]=0; Order[1]=0; Order[2]=0; Order[3]=0;
    /*Make 1e order derivatives in x and y direction  */
    Tm[0] = min( Tpatch_2_3 , Tpatch_4_3); if(IsFinite(Tm[0])){ Order[0]=1; }
    Tm[1] = min( Tpatch_3_2 , Tpatch_3_4); if(IsFinite(Tm[1])){ Order[1]=1; }
    /*Make 1e order derivatives in cross directions  */
    if(usecross) {
        Tm[2] = min( Tpatch_2_2 , Tpatch_4_4); if(IsFinite(Tm[2])){ Order[2]=1; }
        Tm[3] = min( Tpatch_2_4 , Tpatch_4_2); if(IsFinite(Tm[3])){ Order[3]=1; }
    }
	
    /*Make 2e order derivatives  */
    if(usesecond) {
        /*Get Second order derivatives (only use frozen pixel) */
        in=i-2; jn=j+0; if(isfrozen2d(in, jn, dims, Frozen)) { Txm2=T[mindex2(in, jn, dims[0])]; } else { Txm2=INF; }
        in=i+2; jn=j+0; if(isfrozen2d(in, jn, dims, Frozen)) { Txp2=T[mindex2(in, jn, dims[0])]; } else { Txp2=INF; }
        in=i+0; jn=j-2; if(isfrozen2d(in, jn, dims, Frozen)) { Tym2=T[mindex2(in, jn, dims[0])]; } else { Tym2=INF; }
        in=i+0; jn=j+2; if(isfrozen2d(in, jn, dims, Frozen)) { Typ2=T[mindex2(in, jn, dims[0])]; } else { Typ2=INF; }
        if(usecross) {
            in=i-2; jn=j-2; if(isfrozen2d(in, jn, dims, Frozen)) { Tr1m2=T[mindex2(in, jn, dims[0])]; } else { Tr1m2=INF; }
            in=i-2; jn=j+2; if(isfrozen2d(in, jn, dims, Frozen)) { Tr2m2=T[mindex2(in, jn, dims[0])]; } else { Tr2m2=INF; }
            in=i+2; jn=j-2; if(isfrozen2d(in, jn, dims, Frozen)) { Tr2p2=T[mindex2(in, jn, dims[0])]; } else { Tr2p2=INF; }
            in=i+2; jn=j+2; if(isfrozen2d(in, jn, dims, Frozen)) { Tr1p2=T[mindex2(in, jn, dims[0])]; } else { Tr1p2=INF; }
        }
        
        Tm2[0]=0; Tm2[1]=0;Tm2[2]=0; Tm2[3]=0;
        /*pixels with a pixeldistance 2 from the center must be */
        /*lower in value otherwise use other side or first order */
        ch1=(Txm2<Tpatch_2_3)&&IsFinite(Tpatch_2_3); ch2=(Txp2<Tpatch_4_3)&&IsFinite(Tpatch_4_3);
        if(ch1&&ch2) {
            Tm2[0] =min( (4.0*Tpatch_2_3-Txm2)/3.0 , (4.0*Tpatch_4_3-Txp2)/3.0);  Order[0]=2;
        }
        else if (ch1) {
            Tm2[0]=(4.0*Tpatch_2_3-Txm2)/3.0; Order[0]=2;
        }
        else if(ch2) {
            Tm2[0] =(4.0*Tpatch_4_3-Txp2)/3.0; Order[0]=2;
        }
        
        ch1=(Tym2<Tpatch_3_2)&&IsFinite(Tpatch_3_2); ch2=(Typ2<Tpatch_3_4)&&IsFinite(Tpatch_3_4);
        
        if(ch1&&ch2) {
            Tm2[1] =min( (4.0*Tpatch_3_2-Tym2)/3.0 , (4.0*Tpatch_3_4-Typ2)/3.0); Order[1]=2;
        }
        else if(ch1) {
            Tm2[1]=(4.0*Tpatch_3_2-Tym2)/3.0; Order[1]=2;
        }
        else if(ch2) {
            Tm2[1]=(4.0*Tpatch_3_4-Typ2)/3.0; Order[1]=2;
        }
        if(usecross) {
            ch1=(Tr1m2<Tpatch_2_2)&&IsFinite(Tpatch_2_2); ch2=(Tr1p2<Tpatch_4_4)&&IsFinite(Tpatch_4_4);
            if(ch1&&ch2) {
                Tm2[2] =min( (4.0*Tpatch_2_2-Tr1m2)/3.0 , (4.0*Tpatch_4_4-Tr1p2)/3.0); Order[2]=2;
            }
            else if(ch1) {
                Tm2[2]=(4.0*Tpatch_2_2-Tr1m2)/3.0; Order[2]=2;
            }
            else if(ch2){
                Tm2[2]=(4.0*Tpatch_4_4-Tr1p2)/3.0; Order[2]=2;
            }
            
            ch1=(Tr2m2<Tpatch_2_4)&&IsFinite(Tpatch_2_4); ch2=(Tr2p2<Tpatch_4_2)&&IsFinite(Tpatch_4_2);
            if(ch1&&ch2){
                Tm2[3] =min( (4.0*Tpatch_2_4-Tr2m2)/3.0 , (4.0*Tpatch_4_2-Tr2p2)/3.0); Order[3]=2;
            }
            else if(ch1) {
                Tm2[3]=(4.0*Tpatch_2_4-Tr2m2)/3.0; Order[3]=2;
            }
            else if(ch2) {
                Tm2[3]=(4.0*Tpatch_4_2-Tr2p2)/3.0; Order[3]=2;
            }
        }
    }
    /*Calculate the distance using x and y direction */
    Coeff[0]=0; Coeff[1]=0; Coeff[2]=-1/(max(pow2(Fij),eps));
    
    for (t=0; t<2; t++) {
        switch(Order[t]) {
            case 1:
                Coeff[0]+=1; Coeff[1]+=-2*Tm[t]; Coeff[2]+=pow2(Tm[t]);
                break;
            case 2:
                Coeff[0]+=(2.2500); Coeff[1]+=-2.0*Tm2[t]*(2.2500); Coeff[2]+=pow2(Tm2[t])*(2.2500);
                break;
        }
    }
    roots(Coeff, ansroot);
    Tt=max(ansroot[0], ansroot[1]);
    /*Calculate the distance using the cross directions */
    if(usecross) {
        /* Original Equation */
        /*    Coeff[0]=0; Coeff[1]=0; Coeff[2]=-1/(max(pow2(Fij),eps)) */
        Coeff[0]+=0; Coeff[1]+=0; Coeff[2]+=-1/(max(pow2(Fij),eps));
        for (t=2; t<4; t++) {
            switch(Order[t]) {
                case 1:
                    Coeff[0]+=c1; Coeff[1]+=-2.0*c1*Tm[t]; Coeff[2]+=c1*pow2(Tm[t]);
                    break;
                case 2:
                    Coeff[0]+=c1*2.25; Coeff[1]+=-2*c1*Tm2[t]*(2.25); Coeff[2]+=pow2(Tm2[t])*c1*2.25;
                    break;
            }
        }
        if(Coeff[0]>0) {
            roots(Coeff, ansroot);
            Tt2=max(ansroot[0], ansroot[1]);
            /*Select minimum distance value of both stensils */
            Tt=min(Tt, Tt2);
        }
    }
    /*Upwind condition check, current distance must be larger */
    /*then direct neighbours used in solution */
    /*(Will this ever happen?) */
    if(usecross) {
        for(q=0; q<4; q++) { 
            if(IsFinite(Tm[q])&&(Tt<Tm[q])) 
            { 
                Tt=Tm[minarray(Tm, 4)]+(1/(max(Fij,eps)));
            }
        }
    }
    else {
        for(q=0; q<2; q++)
        { 
            if(IsFinite(Tm[q])&&(Tt<Tm[q])) {
                Tt=Tm[minarray(Tm, 2)]+(1/(max(Fij,eps)));}
        }
    }
    return Tt;
}

/* The matlab mex funtion  */
float* main_msfm(float* F, float* source_points, float* T, int* size_map, int* size_target) {
     /* The input variables */;
    bool usesecond = true;  // Default values
    bool usecross = true;   // Default values

    
    /* Euclidian distance image */
    float *Y;
    
    /* Current distance values */
    float Tt, Ty;
    
    /* Matrix containing the Frozen Pixels" */
    bool *Frozen;
    
    /* Augmented Fast Marching (For skeletonize) */
    bool Ed;
    
    /* Size of input image */
    
    int dims[2];
    
    /* Size of  SourcePoints array */
    
    size_t dims_sp[2];

    dims[0] = size_map[0];
    dims[1] = size_map[1];
    dims_sp[0] = size_target[0];
    dims_sp[1] = size_target[1];
    
    /* Number of pixels in image */
    int npixels;
    
    /* Neighbour list */
    int neg_free;
    int neg_pos;
    float *neg_listv;
    float *neg_listx;
    float *neg_listy;
    float *neg_listo;
    
    int *listprop;
    float **listval;
    
    /* Neighbours 4x2 */
    int ne[8]={-1, 1, 0, 0, 0, 0, -1, 1};

    /* Loop variables  */
    int z, k, itt, q;
    
    /* Current location */
    int x, y, i, j;
    
    /* Index */
    int IJ_index, XY_index, index;
    
    
    npixels=size_map[0]*size_map[1];
    
    /* Allocate memory for the distance image. IT MAY NOT BE NEEDED, if the memory is allocated before running the function */
   
    //T = (float *)malloc(npixels * sizeof(float)); 

    if(Ed) { 
        Y = (float *)malloc(npixels * sizeof(float));
    }
    
    /* Pixels which are processed and have a final distance are frozen */
    Frozen = (bool*)malloc( npixels* sizeof(bool) );
    for(q=0;q<npixels;q++){Frozen[q]=0; T[q]=-1;}
    if(Ed)
    {
    for(q=0;q<npixels;q++){Y[q]=-1;}
    }
    
    /*Free memory to store neighbours of the (segmented) region */
    neg_free = 100000;
    neg_pos=0;
    neg_listx = (float *)malloc( neg_free*sizeof(float) );
    neg_listy = (float *)malloc( neg_free*sizeof(float) );
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
    /*(There are 3 pixel classes:
     *  - frozen (processed)
     *  - narrow band (boundary) (in list to check for the next pixel with smallest distance)
     *  - far (not yet used)
     */
    
    /* set all starting points to distance zero and frozen  */
    /* and add all neighbours of the starting points to narrow list  */
    for (z=0; z<dims_sp[1]; z++) {
        /*starting point  */
        x= (int)source_points[0+z*2]-1;
        y= (int)source_points[1+z*2]-1;
        XY_index=x+y*dims[0];
        
        /*Set starting point to frozen and distance to zero  */
        Frozen[XY_index]=1;
        T[XY_index]=0;
        if(Ed) { Y[XY_index]=0; }
    }
    
    for (z=0; z<dims_sp[1]; z++) {
        /*starting point  */
        x= (int)source_points[0+z*2]-1;
        y= (int)source_points[1+z*2]-1;
        XY_index=x+y*dims[0];
        
        /* Add neigbours of starting points  */
        for (k=0; k<4; k++) {
            /*Location of neighbour  */
            i=x+ne[k]; j=y+ne[k+4];
            IJ_index=i+j*dims[0];
            
            /*Check if current neighbour is not yet frozen and inside the
             *picture  */
            if(isntfrozen2d(i, j, dims, Frozen)) {
                Tt=(1/(max(F[IJ_index],eps)));
					
                Ty=1;
                /*Update distance in neigbour list or add to neigbour list */
                if(T[IJ_index]>0) {
                    if(neg_listv[(int)T[IJ_index]]>Tt) {
                        listupdate(listval, listprop, (int)T[IJ_index], Tt);
                    }
                    if(Ed)
                    {
                        neg_listo[(int)T[IJ_index]]=min(neg_listo[(int)T[IJ_index]],Ty);
                    }
                }
                else {
                    /*If running out of memory at a new block  */
                    if(neg_pos>=neg_free) {
                        neg_free+=100000;
                        neg_listx = (float *)realloc(neg_listx, neg_free*sizeof(float) );
                        neg_listy = (float *)realloc(neg_listy, neg_free*sizeof(float) );
                        if(Ed) {
                            neg_listo = (float *)realloc(neg_listo, neg_free*sizeof(float) );
                        }
                    }
                    list_add(listval, listprop, Tt);
                    neg_listv=listval[listprop[1]-1];
                    neg_listx[neg_pos]=i;
                    neg_listy[neg_pos]=j;
                    if(Ed){
                        neg_listo[neg_pos]=Ty;
                    }
                    T[IJ_index]=neg_pos;
                    neg_pos++;
                }
            }
        }
    }
    /*Loop through all pixels of the image  */
    for (itt=0; itt<npixels; itt++) {
        /*Get the pixel from narrow list (boundary list) with smallest
         *distance value and set it to current pixel location  */
        index=list_minimum(listval, listprop);
        neg_listv=listval[listprop[1]-1];
 		
        /* Stop if pixel distance is infinite (all pixels are processed)  */
        if(IsInf(neg_listv[index])) {  break; }
        x=(int)neg_listx[index]; y=(int)neg_listy[index];
        
        XY_index=x+y*dims[0];
        Frozen[XY_index]=1;
        T[XY_index]=neg_listv[index];
        if(Ed) { Y[XY_index]=neg_listo[index]; }
      
     
        /*Remove min value by replacing it with the last value in the array  */
        list_remove_replace(listval, listprop, index) ;
        neg_listv=listval[listprop[1]-1];
        if(index<(neg_pos-1)) {
            neg_listx[index]=neg_listx[neg_pos-1];
            neg_listy[index]=neg_listy[neg_pos-1];
            if(Ed){
                neg_listo[index]=neg_listo[neg_pos-1];
            }
            T[(int)(neg_listx[index]+neg_listy[index]*dims[0])]=index;
        }
        neg_pos =neg_pos-1;
       
    
        /*Loop through all 4 neighbours of current pixel  */
        for (k=0;k<4;k++) {
            
            /*Location of neighbour  */
            i=x+ne[k]; j=y+ne[k+4];
            IJ_index=i+j*dims[0];

            /*Check if current neighbour is not yet frozen and inside the  */
            /*picture  */
            if(isntfrozen2d(i, j, dims, Frozen)) {
				
                Tt=CalculateDistance(T, F[IJ_index], dims, i, j, usesecond, usecross, Frozen);
				        
				if(Ed) {
                    Ty=CalculateDistance(Y, 1, dims, i, j, usesecond, usecross, Frozen);
                }

                /*Update distance in neigbour list or add to neigbour list */
                IJ_index=i+j*dims[0];
                if((T[IJ_index]>-1)&&T[IJ_index]<=listprop[0]) {
                    if(neg_listv[(int)T[IJ_index]]>Tt) {
                        listupdate(listval, listprop,    (int)T[IJ_index], Tt);
                    }
                    if(Ed)
                    {
                        neg_listo[neg_pos]=min(neg_listo[neg_pos],Ty);
                    }
                }
                else {
                    /*If running out of memory at a new block */
                    if(neg_pos>=neg_free) {
                        neg_free+=100000;
                        neg_listx = (float *)realloc(neg_listx, neg_free*sizeof(float) );
                        neg_listy = (float *)realloc(neg_listy, neg_free*sizeof(float) );
                        if(Ed) {
                            neg_listo = (float *)realloc(neg_listo, neg_free*sizeof(float) );
                        }
                    }
                    list_add(listval, listprop, Tt);
                    neg_listv=listval[listprop[1]-1];
                    neg_listx[neg_pos]=i; neg_listy[neg_pos]=j;
                    if(Ed) {
                        neg_listo[neg_pos]=Ty;
                    }
                    T[IJ_index]=neg_pos;
                    neg_pos++;
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
    if(Ed) {
        free(neg_listo);
    }
    free(Frozen);
    free(Y);
}

float* velocities_map(float* binary_map, int* size_map, int threshold, float safety_margin) {
    // Creates the velocities map from the binary occupational map.
    //2D only
    int rows = size_map[1];
    int cols = size_map[0];
    float* distance_map = malloc(rows * cols * sizeof(float));
    float max_distance = sqrt(rows*rows + cols*cols);  // diagonal distance

    // First pass: mark obstacles as 0 and other cells as infinity
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (binary_map[j + i * cols] == 1) {  // obstacle
                distance_map[j + i * cols] = 0.0;
            } else {
                distance_map[j + i * cols] = threshold;
            }
        }
    }
    
    // Create the kernel to be applied to the distance map
    int size_kernel = (threshold*2) + 1;
    int center = size_kernel / 2;
    float* kernel = malloc(size_kernel * size_kernel * sizeof(float));


    for (int i = 0; i < size_kernel; i++) {
        for (int j = 0; j < size_kernel; j++) {
            float dx = (float)(i - center);
            float dy = (float)(j - center);
            kernel[j + i * size_kernel] = sqrt((dx*dx + dy*dy));
        }
    }
    
    // Apply kernel only to obstacles and the cells surrounding them
    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (distance_map[j + i*cols] == 0){
                // Apply kernel to surrounding cells
                for (int ki = -threshold; ki <= threshold; ki++) {
                    for (int kj = -threshold; kj <= threshold; kj++) {
                        // Calculate target cell coordinates
                        int target_i = i + ki;
                        int target_j = j + kj;

                        // Check if target cell is inside the map
                        if (target_i >= 0 && target_i < rows && target_j >= 0 && target_j < cols) {
                            // Calculate kernel index
                            int kernel_i = ki + center;
                            int kernel_j = kj + center;
                            // Apply kernel to target cell
                            float new_dist = kernel[kernel_j + kernel_i * size_kernel];
                            float current_dist = distance_map[target_j + target_i * cols];
                            if (new_dist < current_dist) {
                                distance_map[target_j + target_i * cols] = new_dist;
                            }
                        }
                    }
                }                       
            }
        }
    }
    // Convert distances to velocities using threshold
    
    for (int i = 0; i < rows * cols; i++) {
        if (distance_map[i] != 0.0) {
            // Normalize and apply threshold to create smooth gradient
            float normalized_dist = distance_map[i] / threshold;
            
            if (normalized_dist > 1.0) {
                distance_map[i] = 1.0;  // Maximum velocity
            } else {
                // Create smooth gradient between 0 and 1
                distance_map[i] = normalized_dist;
            }
        }
    }

    free(kernel); 
    return distance_map;
}

void compute_gradient_2d_discrete(float* input_matrix, float* gradient_matrix, int* size_map) {
    
    int rows = size_map[1];
    int cols = size_map[0];
    // 8 neighborhood directions
    const int Ne[8][2] = {
        {-1, -1}, {-1, 0}, {-1, 1},
        { 0, -1},          { 0, 1},
        { 1, -1}, { 1, 0}, { 1, 1}
    };
    
    // For each point in the matrix, including edges
    for(int i = 0; i < rows; i++) {
        for(int j = 0; j < cols; j++) {
            float current_value = input_matrix[i*cols + j];
            float min_value = current_value;
            float fx = 0, fy = 0;
            int valid_neighbors = 0;
            
            // Check all 8 neighbors
            for(int n = 0; n < 8; n++) {
                int ni = i + Ne[n][0];
                int nj = j + Ne[n][1];
                
                // Only process neighbor if it's within bounds
                if(ni >= 0 && ni < rows && nj >= 0 && nj < cols) {
                    valid_neighbors++;
                    float neighbor_value = input_matrix[ni*cols + nj];
                    
                    if(neighbor_value < min_value) {
                        min_value = neighbor_value;
                        // Normalize direction vector
                        float norm = sqrt(Ne[n][0]*Ne[n][0] + Ne[n][1]*Ne[n][1]);
                        fx = Ne[n][1]/norm; // x component in the columns direction
                        fy = Ne[n][0]/norm; // y component in the rows direction
                    }
                }
            }
            
            // Only store gradient if we found valid neighbors
            if(valid_neighbors > 0) {
                gradient_matrix[i*cols + j] = fx;
                gradient_matrix[i*cols + j + rows*cols] = fy;
            } else {
                // No valid neighbors, set gradient to 0
                gradient_matrix[i*cols + j] = 0;
                gradient_matrix[i*cols + j + rows*cols] = 0;
            }
        }
    }
}

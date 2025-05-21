#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include "math.h"

#include "../common.h"
#include "3D_map.h"

float* velocities_map3D(float* binary_map, int* size_map, int threshold) {
    int ancho = size_map[0];
    int largo = size_map[1];
    int alto = size_map[2];
    int slice_size = ancho * largo;
    float* distance_map = malloc(ancho * largo * alto * sizeof(float));
    float max_val = sqrt(ancho*ancho + largo*largo + alto*alto);

    //printf("Calculating distance map\n");
    //First pass: marks obstacles as 0 and other cells as infinity
    for (int k = 0; k < alto; k++) {
        for (int i = 0; i < ancho; i++) {
            for (int j = 0; j < largo; j++) {
                int current_idx = j + i*largo + k*slice_size;
                if(binary_map[current_idx] ==1) {
                    distance_map[current_idx] = 0.0;
                } else {
                    distance_map[current_idx] = max_val;
                }
            }
        }
    }
    
    // Create the 3D kernel

    int size_kernel = (threshold*2) + 1;
    int center = size_kernel / 2;
    float* kernel = malloc(size_kernel * size_kernel * size_kernel * sizeof(float));
    for (int k = 0; k < size_kernel; k++) {
        for (int i = 0; i < size_kernel; i++) {
            for (int j = 0; j < size_kernel; j++) {
                float dx = (float)(i - center);
                float dy = (float)(j - center);
                float dz = (float)(k - center);
                kernel[j + i*size_kernel + k*size_kernel*size_kernel] = sqrt(dx*dx + dy*dy + dz*dz);
                
            }
        }
    }

    // Apply kernel to the cells and update the distance map
    for (int k = 0; k < alto; k++) {
        for (int i = 0; i < ancho; i++) {
            for (int j = 0; j < largo; j++) {
                if (distance_map[j + i*largo + k*slice_size] == 0){
                    for (int ki = -threshold; ki <= threshold; ki++) {
                        for (int kj = -threshold; kj <= threshold; kj++) {
                            for (int kk = -threshold; kk <= threshold; kk++) {
                                // Calculate cell coordinates
                                int target_i = i + ki;
                                int target_j = j + kj;
                                int target_k = k + kk;

                                // Check if target cell is inside the map
                                if (target_i >= 0 && target_i < ancho && target_j >= 0 && target_j < largo &&
                                    target_k >= 0 && target_k < alto) {
                                    // Kernel index
                                    int kernel_i = ki + center;
                                    int kernel_j = kj + center;
                                    int kernel_k = kk + center;
                                    // Apply kernel to target cell
                                    float new_dist = kernel[kernel_j + kernel_i * size_kernel + kernel_k * size_kernel * size_kernel];
                                    float current_dist = distance_map[target_j + target_i*largo + target_k*slice_size];
                                    if (new_dist < current_dist) {
                                        distance_map[target_j + target_i*largo + target_k*slice_size] = new_dist;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    // Covert distances to velocities using threshold
    for (int i = 0; i < slice_size*alto; i++) {
        // Normalize and apply threshold to create smooth gradient
        float normalized_dist = distance_map[i] / threshold;
                    
        if (normalized_dist > 1.0) {
            distance_map[i] = 1.0;  // Maximum velocity
        } else {
            // Create smooth gradient between 0 and 1
            distance_map[i] = normalized_dist;
        }
    }

    return distance_map;
    free(kernel);
}

float* restrictions3D(float* viscosity_map, int* size_map, char* dir){

    // Import/create the soft/hard restrictions map and apply it to the viscosity map

    if (dir != NULL) {
        // Load the restrictions map from the file
        FILE* file = fopen(dir, "rb");
        if (file == NULL) {
            printf("Error: Unable to open restrictions map file.\n");
            return NULL;
        }
        
        // Read the restrictions map from the file
        float* restrictions_map = malloc(size_map[0] * size_map[1] * size_map[2] * sizeof(float));
        if (restrictions_map == NULL) {
            printf("Error: Memory allocation failed for restrictions map.\n");
            fclose(file);
            return NULL;
        }
        fread(restrictions_map, sizeof(float), size_map[0] * size_map[1] * size_map[2], file);
        fclose(file);
        
        // Apply the restrictions map to the viscosity map
        for (int i = 0; i < size_map[0] * size_map[1] * size_map[2]; i++) {
            viscosity_map[i] *= restrictions_map[i];
        }
        
        return viscosity_map;
        free(restrictions_map);
    }
    else{
        // If no restrctions map is provided we can create one
        // Types of restricctions
        printf("No restrictions so far\n");
    }
}
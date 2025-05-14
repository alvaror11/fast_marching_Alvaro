#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include "math.h"
#include "2D_map.h"
#include "../common.h"


float* velocities_map(float* binary_map, int* size_map, int threshold) {
    // Creates the velocities map from the binary occupational map.
    //2D only
    int rows = size_map[1];
    int cols = size_map[0];
    float* distance_map = malloc(rows * cols * sizeof(float));
    
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


float* restrictions2D(float* viscosity_map, int* size_map, char* dir){

    // Import/create the soft/hard restrictions map and apply it to the viscosity map

    if (dir != NULL) {
        // Load the restrictions map from the file
        FILE* file = fopen(dir, "rb");
        if (file == NULL) {
            printf("Error: Unable to open restrictions map file.\n");
            return NULL;
        }
        
        // Read the restrictions map from the file
        float* restrictions_map = malloc(size_map[0] * size_map[1] * sizeof(float));
        if (restrictions_map == NULL) {
            printf("Error: Memory allocation failed for restrictions map.\n");
            fclose(file);
            return NULL;
        }
        fread(restrictions_map, sizeof(float), size_map[0] * size_map[1], file);
        fclose(file);
        
        // Apply the restrictions map to the viscosity map
        for (int i = 0; i < size_map[0] * size_map[1]; i++) {
            viscosity_map[i] *= restrictions_map[i];
        }
        
        return viscosity_map;
        free(restrictions_map);
    }
    else{
        // If no restrctions map is provided we can create one
        // Types of restricctions
        printf("no restrictions so far\n");

    }


}
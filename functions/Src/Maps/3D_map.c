#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include "math.h"

#include "../common.h"
#include "3D_map.h"

void apply_restrictions3D(float* restrictions_map, int* size_map, float restriction_type, int (*areas)[6], int num_areas) {
    // Loop through each area
    for (int area_idx = 0; area_idx < num_areas; area_idx++) {
        // Get current area bounds
        int x_min = areas[area_idx][0] - 1;
        int x_max = areas[area_idx][1] - 1;
        int y_min = areas[area_idx][2] - 1;
        int y_max = areas[area_idx][3] - 1;
        int z_min = areas[area_idx][4] - 1;
        int z_max = areas[area_idx][5] - 1;

         // Check if area is completely within boundaries
        if (x_min < 0 || x_max >= size_map[0] || 
            y_min < 0 || y_max >= size_map[1] ||
            z_min < 0 || z_max >= size_map[2]) {
        
            printf("Warning: Area %d [%d,%d,%d,%d,%d,%d] is outside map boundaries [%d,%d,%d]. Skipping.\n", 
                   area_idx, x_min+1, x_max+1, y_min+1, y_max+1, z_min+1, z_max+1,
                   size_map[0], size_map[1], size_map[2]);
            continue;  // Skip this area
        }

        // Check if area boundaries make sense
        if (x_min > x_max || y_min > y_max || z_min > z_max) {
            printf("Warning: Area %d has invalid bounds [%d,%d,%d,%d,%d,%d]. Skipping.\n",
                   area_idx, x_min+1, x_max+1, y_min+1, y_max+1, z_min+1, z_max+1);
            continue;  // Skip this area
        }

        // Apply restriction value to all points in this area
        for (int x = x_min; x <= x_max; x++) {
            for (int y = y_min; y <= y_max; y++) {
                for (int z = z_min; z <= z_max; z++) {
                int index = y + x * size_map[1] + z * size_map[0] * size_map[1];
                restrictions_map[index] = restriction_type;
                }
            }
        }
    }
   
}


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

float* restrictions3D(float* viscosity_map, int* size_map, char* dir, 
                      float* objective_points, int* size_objective, float* start_points, int size_start[2]){

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
        float hard_restrictions = 0;
        float soft_restrictions1 = 0.2;
        float soft_restrictions2 = 0.6;
        float soft_restrictions3 = 0.75;

        // Define number of areas for each restriction type
        #define NUM_HARD_AREAS 1
        #define NUM_SOFT1_AREAS 1
        #define NUM_SOFT2_AREAS 1
        #define NUM_SOFT3_AREAS 1

         // Define areas [x_min, x_max, y_min, y_max, z_min, z_max] for each restriction type
        int hard_areas[NUM_HARD_AREAS][6] = {
            {1, 200, 1, 200, 150, 200}     
        };

        int soft1_areas[NUM_SOFT1_AREAS][6] = {
            {30, 60, 30, 60, 10, 20}     
        };

        int soft2_areas[NUM_SOFT2_AREAS][6] = {
            {1, 40, 1, 40, 40, 50}
        };

        int soft3_areas[NUM_SOFT3_AREAS][6] = {
            {10, 40, 10, 40, 25, 50}
        };

        //Check if objective or start points are inside a hard restriction area
        for (int p = 0; p < size_objective[0]; p++) {
            int x = (int)objective_points[p*3] - 1;
            int y = (int)objective_points[p*3 + 1] - 1;
            int z = (int)objective_points[p*3 + 2] - 1;
            
            for (int a = 0; a < NUM_HARD_AREAS; a++) {
                if (x >= hard_areas[a][0]-1 && x <= hard_areas[a][1]-1 &&
                    y >= hard_areas[a][2]-1 && y <= hard_areas[a][3]-1 &&
                    z >= hard_areas[a][4]-1 && z <= hard_areas[a][5]-1) {
                    printf("Warning: Objective point %d [%d,%d,%d] is inside hard restriction area %d!\n",
                        p+1, x+1, y+1, z+1, a+1);
                    return NULL;
                }
            }
        }
        for (int p = 0; p < size_start[0]; p++) {
            int x = (int)start_points[p*3] - 1;
            int y = (int)start_points[p*3 + 1] - 1;
            int z = (int)start_points[p*3 + 2] - 1;
            
            for (int a = 0; a < NUM_HARD_AREAS; a++) {
                if (x >= hard_areas[a][0]-1 && x <= hard_areas[a][1]-1 &&
                    y >= hard_areas[a][2]-1 && y <= hard_areas[a][3]-1 &&
                    z >= hard_areas[a][4]-1 && z <= hard_areas[a][5]-1) {
                    printf("Warning: Start point %d [%d,%d,%d] is inside hard restriction area %d!\n",
                        p+1, x+1, y+1, z+1, a+1);
                    return NULL;
                }
            }
        }


        float* restrictions_map = malloc(size_map[0] * size_map[1] * size_map[2] * sizeof(float));
        if (restrictions_map == NULL) {
            printf("Error: Memory allocation failed for restrictions map.\n");
            return NULL;
        }

        // Fill the matrix with 1s (no restrictions)
        for (int i = 0; i < size_map[0] * size_map[1] * size_map[2]; i++) {
            restrictions_map[i] = 1.0f;
        }

        if (NUM_SOFT3_AREAS > 0){
            // Apply soft restrictions
            apply_restrictions3D(restrictions_map, size_map, soft_restrictions3, soft3_areas, NUM_SOFT3_AREAS);
        }
         if (NUM_SOFT2_AREAS > 0){
            // Apply soft restrictions
            apply_restrictions3D(restrictions_map, size_map, soft_restrictions2, soft2_areas, NUM_SOFT2_AREAS);
        }
        if (NUM_SOFT1_AREAS > 0){
            // Apply soft restrictions
            apply_restrictions3D(restrictions_map, size_map, soft_restrictions1, soft1_areas, NUM_SOFT1_AREAS);
        }
        if (NUM_HARD_AREAS > 0){
            // Apply hard restrictions
            apply_restrictions3D(restrictions_map, size_map, hard_restrictions, hard_areas, NUM_HARD_AREAS);
        }

        // Apply to viscosity map (multiplication by 1 won't change values)
        for (int i = 0; i < size_map[0] * size_map[1] * size_map[2]; i++) {
            viscosity_map[i] *= restrictions_map[i];
        }

        return viscosity_map;
        free(restrictions_map);

    }
}
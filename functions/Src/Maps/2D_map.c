#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stddef.h>
#include "math.h"

#include "2D_map.h"
#include "../common.h"

void apply_restrictions(float* restrictions_map, int* size_map, float restriction_type, int (*areas)[4], int num_areas) {
    // Loop through each area
    for (int area_idx = 0; area_idx < num_areas; area_idx++) {
        // Get current area bounds
        int x_min = areas[area_idx][0] - 1;
        int x_max = areas[area_idx][1] - 1;
        int y_min = areas[area_idx][2] - 1;
        int y_max = areas[area_idx][3] - 1;

         // Check if area is completely within boundaries
        if (x_min < 0 || x_max >= size_map[0] || 
            y_min < 0 || y_max >= size_map[1]) {
            printf("Warning: Area %d [%d,%d,%d,%d] is outside map boundaries [%d,%d]. Skipping.\n", 
                   area_idx, x_min+1, x_max+1, y_min+1, y_max+1, 
                   size_map[0], size_map[1]);
            continue;  // Skip this area
        }

        // Check if area boundaries make sense
        if (x_min > x_max || y_min > y_max) {
            printf("Warning: Area %d has invalid bounds [%d,%d,%d,%d]. Skipping.\n",
                   area_idx, x_min+1, x_max+1, y_min+1, y_max+1);
            continue;  // Skip this area
        }

        // Apply restriction value to all points in this area
        for (int x = x_min; x <= x_max; x++) {
            for (int y = y_min; y <= y_max; y++) {
                int index = x + y * size_map[0];
                restrictions_map[index] = restriction_type;
            }
        }
    }
   
}


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


float* restrictions2D(float* viscosity_map, int* size_map, char* dir,
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
        float hard_restrictions = 0;
        float soft_restrictions1 = 0.2;
        float soft_restrictions2 = 0.6;
        float soft_restrictions3 = 0.75;


        // Define number of areas for each restriction type
        #define NUM_HARD_AREAS 1
        #define NUM_SOFT1_AREAS 1
        #define NUM_SOFT2_AREAS 1
        #define NUM_SOFT3_AREAS 1

        // Define areas [x_min, x_max, y_min, y_max] for each restriction type
        int hard_areas[NUM_HARD_AREAS][4] = {
            {20, 30, 20, 30}     
        };

        int soft1_areas[NUM_SOFT1_AREAS][4] = {
            {30, 60, 30, 60}     
        };

        int soft2_areas[NUM_SOFT2_AREAS][4] = {
            {1, 40, 1, 40}
        };

        int soft3_areas[NUM_SOFT3_AREAS][4] = {
            {10, 40, 10, 40}
        };

        //Check if objective or start points are inside a hard restriction area
        for (int p = 0; p < size_objective[0]; p++) {
            int x = (int)objective_points[p*3] - 1;
            int y = (int)objective_points[p*3 + 1] - 1;
            
            for (int a = 0; a < NUM_HARD_AREAS; a++) {
                if (x >= hard_areas[a][0]-1 && x <= hard_areas[a][1]-1 &&
                    y >= hard_areas[a][2]-1 && y <= hard_areas[a][3]-1) {
                    printf("Warning: Objective point %d [%d,%d,%d] is inside hard restriction area %d!\n",
                        p+1, x+1, y+1, a+1);
                    return NULL;
                }
            }
        }
        for (int p = 0; p < size_start[0]; p++) {
            int x = (int)start_points[p*3] - 1;
            int y = (int)start_points[p*3 + 1] - 1;
            
            for (int a = 0; a < NUM_HARD_AREAS; a++) {
                if (x >= hard_areas[a][0]-1 && x <= hard_areas[a][1]-1 &&
                    y >= hard_areas[a][2]-1 && y <= hard_areas[a][3]-1) {
                    printf("Warning: Start point %d [%d,%d,%d] is inside hard restriction area %d!\n",
                        p+1, x+1, y+1, a+1);
                    return NULL;
                }
            }
        }




        float* restrictions_map = malloc(size_map[0] * size_map[1] * sizeof(float));
        if (restrictions_map == NULL) {
            printf("Error: Memory allocation failed for restrictions map.\n");
            return NULL;
        }

        // Fill the matrix with 1s (no restrictions)
        for (int i = 0; i < size_map[0] * size_map[1]; i++) {
            restrictions_map[i] = 1.0f;
        }

        if (NUM_SOFT3_AREAS > 0){
            // Apply soft restrictions
            apply_restrictions(restrictions_map, size_map, soft_restrictions3, soft3_areas, NUM_SOFT3_AREAS);
        }
         if (NUM_SOFT2_AREAS > 0){
            // Apply soft restrictions
            apply_restrictions(restrictions_map, size_map, soft_restrictions2, soft2_areas, NUM_SOFT2_AREAS);
        }
        if (NUM_SOFT1_AREAS > 0){
            // Apply soft restrictions
            apply_restrictions(restrictions_map, size_map, soft_restrictions1, soft1_areas, NUM_SOFT1_AREAS);
        }
        if (NUM_HARD_AREAS > 0){
            // Apply hard restrictions
            apply_restrictions(restrictions_map, size_map, hard_restrictions, hard_areas, NUM_HARD_AREAS);
        }

        // Apply to viscosity map (multiplication by 1 won't change values)
        for (int i = 0; i < size_map[0] * size_map[1]; i++) {
            viscosity_map[i] *= restrictions_map[i];
        }

        FILE *output_file2 = fopen("../Archivos/restrictions_map.txt", "w");
        if (output_file2 == NULL) {
            perror("Error al abrir el archivo de salida");
            return NULL;
        }
        
        fprintf(output_file2, "%d %d\n", size_map[0], size_map[1]);

        for (int i = 0; i < size_map[1]; i++) {
            for (int j = 0; j < size_map[0]; j++) {
                fprintf(output_file2, "%.2f ", viscosity_map[j + i * size_map[0]]);
            }
            fprintf(output_file2, "\n");
        }
        fclose(output_file2);

        return viscosity_map;
        free (restrictions_map);

    }


}



float* restrictions25D(float* viscosity_map, int* size_map, char* dir,
                     float* objective_points, int* size_objective, float* start_points, int size_start[2]){
                     
                    
                    

                        


                    
                    
                    
}
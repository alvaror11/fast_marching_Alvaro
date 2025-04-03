#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "common.h"


void planners_2D(double* matriz, int* size_map, double* objective_points, int size_objective[2], 
                double* start_points, int size_start[2], int planner_type, int escalado_vectores) {

    // Choose type of planner
    switch(planner_type){        
        case 0:
            // Use original map
            break;
        case 1:{
            // Coger el cuadrado mas pequeño que contega puntos objetivo y de inicio
            int x_max = 0;
            int y_max = 0;
            int x_min = size_map[0];
            int y_min = size_map[1];
            // Find max and min coordinates for objective points
            for (int i = 0; i < size_objective[1]; i++){
                int x = (int) objective_points[i*2];
                int y = (int) objective_points[i*2 + 1];
                if (x > x_max) x_max = x;
                if (y > y_max) y_max = y;
                if (x < x_min) x_min = x;
                if (y < y_min) y_min = y;
            }

            // Find max and min coordinates for start points
            for (int i = 0; i < size_objective[1]; i++){
                int x = (int) start_points[i*2];
                int y = (int) start_points[i*2 + 1];
                if (x > x_max) x_max = x;
                if (y > y_max) y_max = y;
                if (x < x_min) x_min = x;
                if (y < y_min) y_min = y;
            }
            
            double distance = euclidean_distance(x_max, y_max, x_min, y_min);
            // Calculate margin as 10% of the distance
            int margin = (int)(0.1 * distance);
            if (margin < 2) margin = 2;  // Minimum margin of 2 cells
 
            // Apply margins with boundary checks
            x_min = (x_min - margin >= 0) ? x_min - margin : 0;
            y_min = (y_min - margin >= 0) ? y_min - margin : 0;
            x_max = (x_max + margin < size_map[0]) ? x_max + margin : size_map[0] - 1;
            y_max = (y_max + margin < size_map[1]) ? y_max + margin : size_map[1] - 1;
            
            // Calculate new map dimensions
            int new_cols = x_max - x_min + 1;
            int new_rows = y_max - y_min + 1;

            // LLenar la parte del mapa que no se usa con obstaculos
            for (int i = 0; i < (int)size_map[1]; i++) {
                for (int j = 0; j < (int)size_map[0]; j++) {
                    if (i < y_min || i > y_max || j < x_min || j > x_max) {
                        matriz[i * size_map[0] + j] = 0;
                    }
                }
            }
            

        }
        case 2:{
            // Select a rectangle that connects all objective and start
            int num_start = size_objective[1]; 
            int num_end = size_start[1];    
            
            int x_max = 0;
            int y_max = 0;
            int x_min = size_map[0];
            int y_min = size_map[1];

            for (int i = 0; i<num_start; i++){
                double start_x = start_points[i*2];
                double start_y = start_points[i*2 + 1];

                for (int j = 0; j<num_end; j++){
                    double end_x = objective_points[j*2];
                    double end_y = objective_points[j*2 + 1];

                    //Calculate vector
                    double vector_x = end_x - start_x;
                    double vector_y = end_y - start_y;

                    //Normalize
                    double norm = sqrt(vector_x*vector_x + vector_y*vector_y);
                    vector_x = (vector_x / norm) * escalado_vectores; 
                    vector_y = (vector_y / norm) * escalado_vectores;

                    double normal_x = -vector_y;
                    double normal_y = vector_x;

                    // Check points for start position
                    double points[8][2] = {
                        {start_x + vector_x, start_y + vector_y},   // Forward
                        {start_x - vector_x, start_y - vector_y},   // Backward
                        {start_x + normal_x, start_y + normal_y},   // Right
                        {start_x - normal_x, start_y - normal_y},   // Left
                        {end_x + vector_x, end_y + vector_y},       // Forward
                        {end_x - vector_x, end_y - vector_y},       // Backward
                        {end_x + normal_x, end_y + normal_y},       // Right
                        {end_x - normal_x, end_y - normal_y}        // Left
                    };

                    // Update bounds for all points
                    for (int k = 0; k < 8; k++) {
                        // Apply ceil/floor first then clamp to map boundaries
                        int temp_x = (int)((points[k][0] > x_max) ? ceil(points[k][0]) : floor(points[k][0]));
                        int temp_y = (int)((points[k][1] > y_max) ? ceil(points[k][1]) : floor(points[k][1]));
                        
                        // Clamp values to map boundaries
                        temp_x = (temp_x < 0) ? 0 : ((temp_x >= size_map[0]) ? size_map[0] - 1 : temp_x);
                        temp_y = (temp_y < 0) ? 0 : ((temp_y >= size_map[1]) ? size_map[1] - 1 : temp_y);
                        
                        // Update max/min values
                        if (temp_x > x_max) x_max = temp_x;
                        if (temp_x < x_min) x_min = temp_x;
                        if (temp_y > y_max) y_max = temp_y;
                        if (temp_y < y_min) y_min = temp_y;
                    }                 
                }
            }
            
            // LLenar la parte del mapa que no se usa con obstaculos
            for (int i = 0; i < (int)size_map[1]; i++) {
                for (int j = 0; j < (int)size_map[0]; j++) {
                    if (i < y_min || i > y_max || j < x_min || j > x_max) {
                        matriz[i * size_map[0] + j] = 0;
                    }
                }
            }
            break;

            FILE *file = fopen("reduced_map.txt", "w");
            if (file == NULL) {
                printf("Error opening file for writing\n");
                return;
            }

            // Write the map data
            for (int i = 0; i < (int)size_map[1]; i++) {
                for (int j = 0; j < (int)size_map[0]; j++) {
                    fprintf(file, "%.2f ", matriz[i * (int)size_map[0] + j]);
                }
                fprintf(file, "\n");  // New line after each row
            }

            fclose(file);       
        }   
    }
}

void planners_3D(double* matriz, int* size_map, double* objective_points, int size_objective[2], 
    double* start_points, int size_start[2], int planner_type, int escalado_vectores){
        
    switch(planner_type){        
        case 0:
            // Use original map
            break;

        case 1: {
            // Initialize bounds
            int x_max = 0, y_max = 0, z_max = 0;
            int x_min = size_map[0], y_min = size_map[1], z_min = size_map[2];
            int num_start = size_start[1];
            int num_end = size_objective[1];

            printf("\nCalculating 3D corridor bounds...\n");

            for (int i = 0; i < num_start; i++) {
                double start_x = start_points[i*3];
                double start_y = start_points[i*3 + 1];
                double start_z = start_points[i*3 + 2];

                for (int j = 0; j < num_end; j++) {
                    double end_x = objective_points[j*3];
                    double end_y = objective_points[j*3 + 1];
                    double end_z = objective_points[j*3 + 2];

                    // Calculate directing vector and normalize+scale in one step
                    double vector_x = end_x - start_x;
                    double vector_y = end_y - start_y;
                    double vector_z = end_z - start_z;
                    double norm = sqrt(vector_x*vector_x + vector_y*vector_y + vector_z*vector_z);
                    vector_x = (vector_x / norm) * escalado_vectores;
                    vector_y = (vector_y / norm) * escalado_vectores;
                    vector_z = (vector_z / norm) * escalado_vectores;

                    // First perpendicular vector (already inherits scale from vector)
                    double perp1_x = -vector_y;
                    double perp1_y = vector_x;
                    double perp1_z = 0;

                    // Second perpendicular vector using cross product
                    double perp2_x = vector_y*perp1_z - vector_z*perp1_y;
                    double perp2_y = vector_z*perp1_x - vector_x*perp1_z;
                    double perp2_z = vector_x*perp1_y - vector_y*perp1_x;

                    // Generate points array
                    double points[12][3] = {
                        // Start point expansions
                        {start_x + vector_x, start_y + vector_y, start_z + vector_z},
                        {start_x - vector_x, start_y - vector_y, start_z - vector_z},
                        {start_x + perp1_x, start_y + perp1_y, start_z + perp1_z},
                        {start_x - perp1_x, start_y - perp1_y, start_z - perp1_z},
                        {start_x + perp2_x, start_y + perp2_y, start_z + perp2_z},
                        {start_x - perp2_x, start_y - perp2_y, start_z - perp2_z},
                        // End point expansions
                        {end_x + vector_x, end_y + vector_y, end_z + vector_z},
                        {end_x - vector_x, end_y - vector_y, end_z - vector_z},
                        {end_x + perp1_x, end_y + perp1_y, end_z + perp1_z},
                        {end_x - perp1_x, end_y - perp1_y, end_z - perp1_z},
                        {end_x + perp2_x, end_y + perp2_y, end_z + perp2_z},
                        {end_x - perp2_x, end_y - perp2_y, end_z - perp2_z}
                    };

                    // Update bounds for all points
                    for (int k = 0; k < 12; k++) {
                        
                        int temp_x = (int)((points[k][0] > x_max) ? ceil(points[k][0]) : floor(points[k][0]));
                        int temp_y = (int)((points[k][1] > y_max) ? ceil(points[k][1]) : floor(points[k][1]));
                        int temp_z = (int)((points[k][2] > z_max) ? ceil(points[k][2]) : floor(points[k][2]));
                        
                        // Clamp to map boundaries
                        temp_x = (temp_x < 0) ? 0 : ((temp_x >= size_map[0]) ? size_map[0] - 1 : temp_x);
                        temp_y = (temp_y < 0) ? 0 : ((temp_y >= size_map[1]) ? size_map[1] - 1 : temp_y);
                        temp_z = (temp_z < 0) ? 0 : ((temp_z >= size_map[2]) ? size_map[2] - 1 : temp_z);
                        
                        // Update bounds
                        if (temp_x > x_max) x_max = temp_x;
                        if (temp_x < x_min) x_min = temp_x;
                        if (temp_y > y_max) y_max = temp_y;
                        if (temp_y < y_min) y_min = temp_y;
                        if (temp_z > z_max) z_max = temp_z;
                        if (temp_z < z_min) z_min = temp_z;
                    }
                }
            }

            // Fill map outside bounds with obstacles
            for (int k = 0; k < size_map[2]; k++) {
                for (int i = 0; i < size_map[0]; i++) {
                    for (int j = 0; j < size_map[1]; j++) {
                        if (j < x_min || j > x_max || 
                            i < y_min || i > y_max || 
                            k < z_min || k > z_max) {
                            matriz[j + i*size_map[1] + k*size_map[0]*size_map[1]] = 1;
                        }
                    }
                }
            }
            break;
        }
    }
}
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

        // Common variables
        
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
                        matriz[i * size_map[0] + j] = 1;
                    }
                }
            }
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

            /*
            // Create reduced map
            double* reduced_map = (double*)malloc(new_cols * new_rows * sizeof(double));
            
            // Copy relevant portion of the map
            for (int i = 0; i < new_rows; i++) {
                for (int j = 0; j < new_cols; j++) {
                    reduced_map[i * new_cols + j] = matriz[size_map[0] * (i + x_min) + (j + y_min)];
                }
            }

            // Update objective points coordinates
            for (int i = 0; i < size_objective[1]; i++) {
                objective_points[i*2] -= x_min;
                objective_points[i*2 + 1] -= y_min;
            }

            // Update start points coordinates
            for (int i = 0; i < size_objective[1]; i++) {
                start_points[i*2] -= x_min;
                start_points[i*2 + 1] -= y_min;
            }
            
            matriz = (double*)realloc(matriz, new_cols * new_rows * sizeof(double));
            memcpy(matriz, reduced_map, new_cols * new_rows * sizeof(double));
            free(reduced_map);
            // Update map size
            size_map[0] = new_cols;
            size_map[1] = new_rows;

            FILE *file = fopen("reduced_map.txt", "w");
            if (file == NULL) {
                printf("Error opening file for writing\n");
                return;
            }

            // Write dimensions in the first line
            fprintf(file, "%d %d\n", new_rows, new_cols);

            // Write the map data
            for (int i = 0; i < new_rows; i++) {
                for (int j = 0; j < new_cols; j++) {
                    fprintf(file, "%.2f ", matriz[i * new_cols + j]);
                }
                fprintf(file, "\n");  // New line after each row
            }

            fclose(file);
            */
        }
        case 2:{
            // Select a rectangle that connects all objective and start
            int num_start = size_objective[1]; 
            int num_end = size_start[1];    
            int total_vectors = num_start * num_end;  // All possible combinations
            
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
                        matriz[i * size_map[0] + j] = 1;
                    }
                }
            }
            
            //Save reduced map
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
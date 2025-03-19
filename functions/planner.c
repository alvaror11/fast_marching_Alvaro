#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "common.h"


void planners_2D(double* matriz, int* size_map, double* objective_points, int size_objective[2], 
                double* start_points, int planner_type){

    // Choose type of planner
    switch(planner_type){
        
        case 0:
            // Use original map
            break;
        case 1:
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

}
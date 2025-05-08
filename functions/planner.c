#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "common.h"

#define PI 3.14159265358979323846


void planners_2D(float* matriz, int* size_map, float* objective_points, int size_objective[2], 
                float* start_points, int size_start[2], int planner_type, int escalado_vectores) {

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
            
            float distance = euclidean_distance(x_max, y_max, x_min, y_min);
            // Calculate margin as 10% of the distance
            int margin = (int)(0.1 * distance);
            if (margin < 2) margin = 2;  // Minimum margin of 2 cells
 
            // Apply margins with boundary checks
            x_min = (x_min - margin >= 0) ? x_min - margin : 0;
            y_min = (y_min - margin >= 0) ? y_min - margin : 0;
            x_max = (x_max + margin < size_map[0]) ? x_max + margin : size_map[0] - 1;
            y_max = (y_max + margin < size_map[1]) ? y_max + margin : size_map[1] - 1;

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
                float start_x = start_points[i*2];
                float start_y = start_points[i*2 + 1];

                for (int j = 0; j<num_end; j++){
                    float end_x = objective_points[j*2];
                    float end_y = objective_points[j*2 + 1];

                    //Calculate vector
                    float vector_x = end_x - start_x;
                    float vector_y = end_y - start_y;

                    //Normalize
                    float norm = sqrt(vector_x*vector_x + vector_y*vector_y);
                    vector_x = (vector_x / norm) * escalado_vectores; 
                    vector_y = (vector_y / norm) * escalado_vectores;

                    float normal_x = -vector_y;
                    float normal_y = vector_x;

                    // Check points for start position
                    float points[8][2] = {
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

void planners_3D(float* matriz, int* size_map, float* objective_points, int size_objective[2], 
    float* start_points, int size_start[2], int planner_type, int escalado_vectores, 
    int* height_map, float* occupation_map_2d, int ascension_rate, int descent_rate, int flight_level, int resolution) {
        
    switch(planner_type){        
        case 0:
            // Use original map
            break;

        case 1: {
            // Trinchera vert y horizontal combinada
            int x_max = 0, y_max = 0, z_max = 0;
            int x_min = size_map[0], y_min = size_map[1], z_min = size_map[2];
            int num_start = size_start[1];
            int num_end = size_objective[1];

            for (int i = 0; i < num_start; i++) {
                float start_x = start_points[i*3];
                float start_y = start_points[i*3 + 1];
                float start_z = start_points[i*3 + 2];

                for (int j = 0; j < num_end; j++) {
                    float end_x = objective_points[j*3];
                    float end_y = objective_points[j*3 + 1];
                    float end_z = objective_points[j*3 + 2];

                    // Calculate directing vector and normalize+scale in one step
                    float vector_x = end_x - start_x;
                    float vector_y = end_y - start_y;
                    float vector_z = end_z - start_z;
                    float norm = sqrt(vector_x*vector_x + vector_y*vector_y + vector_z*vector_z);
                    vector_x = (vector_x / norm) * escalado_vectores;
                    vector_y = (vector_y / norm) * escalado_vectores;
                    vector_z = (vector_z / norm) * escalado_vectores;

                    // First perpendicular vector (already inherits scale from vector)
                    float perp1_x = -vector_y;
                    float perp1_y = vector_x;
                    float perp1_z = 0;

                    // Second perpendicular vector using cross product
                    float perp2_x = vector_y*perp1_z - vector_z*perp1_y;
                    float perp2_y = vector_z*perp1_x - vector_x*perp1_z;
                    float perp2_z = vector_x*perp1_y - vector_y*perp1_x;

                    // Generate points array
                    float points[12][3] = {
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
        case 2: {

            int flight_level_cells = (flight_level / resolution) - 1; // Convert height to cells

            // Substract 1 to convert from 1-based to 0-based indexing
            int start_x = start_points[0] - 1;
            int start_y = start_points[1] - 1;
            int start_z = start_points[2] - 1;
            int objective_x = objective_points[0] - 1;
            int objective_y = objective_points[1] - 1;
            int objective_z = objective_points[2] - 1;  

            // Create 2d matrix with height values
            // create the ascension cone
            printf("Creating height map...\n");
            for(int z = start_z; z <= flight_level_cells; z++) {
                // Calculate radius at this height (in cells)
                float height_diff = (z - start_z) * resolution;
                float radius_cells = (height_diff / ascension_rate) / resolution;
                
                // Generate circle at this height
                for(float angle = 0; angle < 2*PI; angle += PI/(32*height_diff)) {
                    int x = start_x + (int)floor(radius_cells * cos(angle));
                    int y = start_y + (int)floor(radius_cells * sin(angle));
                    
                    // Check bounds
                    if(x >= 0 && x < size_map[0] && y >= 0 && y < size_map[1]) {
                        // Update projection if this height is higher
                        int idx = y + size_map[1] * x;
                        if(height_map[idx] == 0 || z < height_map[idx]) {
                            height_map[idx] = z;
                        }
                    }
                }
            }
            // create the decent cone
            for(int z = objective_z; z <= flight_level_cells; z++) {
                float height_diff = (z - objective_z) * resolution;
                float radius_cells = (height_diff / descent_rate) / resolution;
                
                for(float angle = 0; angle < 2*PI; angle += PI/(height_diff*32)) {
                    int x = objective_x + (int)floor(radius_cells * cos(angle));
                    int y = objective_y + (int)floor(radius_cells * sin(angle));
                    
                    if(x >= 0 && x < size_map[0] && y >= 0 && y < size_map[1]) {
                        int idx = y + size_map[1] * x;
                        if(height_map[idx] == 0 || z < height_map[idx]) {
                            height_map[idx] = z;
                        }
                    }
                }
            }

           
            //Fill the rest of the height map with the flight level height
            // Flight Level Cells: written as index in the map, level 20 will be 19 in the map
            for(int i = 0; i < size_map[0]; i++) {
                for(int j = 0; j < size_map[1]; j++) {
                    int idx = j + size_map[1] * i;
                    if(height_map[idx] == 0) {
                        height_map[idx] = flight_level_cells;
                    }
                }
            }

             // Save height map to file
             FILE *height_file = fopen("./Archivos/height_map.txt", "w");
             if (!height_file) {
                 printf("Error opening height map file\n");
                 free(height_map);
                 return;
             } 
             // Write height map data in matrix format
             for(int i = 0; i < size_map[0]; i++) {
                for(int j = 0; j < size_map[1]; j++) {
                     fprintf(height_file, "%d ", height_map[i * size_map[1] + j]);
                 }
                 fprintf(height_file, "\n");
             }
 
             fclose(height_file);


            // Check vertical obstacles for each x,y coordinate
            // Seguimos con coordenadas estilo 3d, i = x, j = y, k = z
            for(int i = 0; i < size_map[0]; i++) {
                for(int j = 0; j < size_map[1]; j++) {
                    int idx_2d = j + size_map[1] * i;
                    float z = height_map[idx_2d];
                    bool is_occupied = false;

                    // Check for obstacles above and below the surface point
                    for(int k = (z - 1); k <  (z + 2); k++) {
                        if (k < 0 || k >= size_map[2]) {
                            continue; // Skip out of bounds
                        }
                        // Check if the cell is occupied
                        int idx_3d = j + i*size_map[1] + k*size_map[0]*size_map[1];
                        if(matriz[idx_3d] == 1.0f) {
                            is_occupied = true;
                            break;
                        }
                    }
                    // Set occupation map value
                    occupation_map_2d[idx_2d] = is_occupied ? 1.0f : 0.0f;
                }
            }
            // Save 2D occupation map to file
            FILE* occupation_file = fopen("./Archivos/occupation_map_2d.txt", "w");
            if (!occupation_file) {
                printf("Error opening 2D occupation map file\n");
                free(height_map);
                free(occupation_map_2d);
                return;
            }
            // Write 2D occupation map data
            for(int i = 0; i < size_map[0]; i++) {
                for(int j = 0; j < size_map[1]; j++) {
                    fprintf(occupation_file, "%.0f ", occupation_map_2d[j + size_map[1]*i]);
                }
                fprintf(occupation_file, "\n");
            }

            fclose(occupation_file);
        }
    }
}
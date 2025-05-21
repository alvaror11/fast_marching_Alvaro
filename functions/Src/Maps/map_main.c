
#include <stddef.h>
#include <stdio.h>
#include <time.h>

#include "2D_map.h"
#include "3D_map.h"
#include "planner.h"
#include "../common.h"

float* map_main2D(float* matriz, int* size_map, int distance_threshold,
                float* objective_points, int* size_objective, float* start_points, int* size_start,
                int planner_type, float escalado_vectores){


        
        //Create velocities map
    printf("Creating velocities map...\n");
    float* obstacle_distance_map = velocities_map(matriz, size_map, distance_threshold);
    //Save velocities map
    FILE *output_file1 = fopen("../Archivos/velocities_map.txt", "w");
     if (output_file1 == NULL) {
         perror("Error al abrir el archivo de salida");
         return NULL;
     }
     
     fprintf(output_file1, "%d %d\n", size_map[0], size_map[1]);

     for (int i = 0; i < size_map[1]; i++) {
         for (int j = 0; j < size_map[0]; j++) {
             fprintf(output_file1, "%.2f ", obstacle_distance_map[j + i * size_map[0]]);
         }
         fprintf(output_file1, "\n");
     }
     fclose(output_file1);

     
     //Apply restrictions
    float* restrictions_map = restrictions2D(obstacle_distance_map, size_map, NULL);

    // Apply planner
    planners_2D(restrictions_map, size_map, objective_points, size_objective, start_points, size_start, planner_type, escalado_vectores);

    return restrictions_map;

    free(matriz);
    free(obstacle_distance_map);
}

float* map_main3D(float* matriz, int* size_map, int distance_threshold,
                float* objective_points, int* size_objective, float* start_points, int* size_start,
                int planner_type, float escalado_vectores){

    //Create velocities map
    printf("Creating velocities map...\n");

    clock_t start_velocitiesMap = clock();
    float* obstacle_distance_map = velocities_map3D(matriz, size_map, distance_threshold);
    clock_t end_velocitiesMap = clock();

    float time_velocitiesMap = ((float) (end_velocitiesMap - start_velocitiesMap)) / CLOCKS_PER_SEC;
    printf("Time for velocities map: %.3f s\n", time_velocitiesMap);

    // Save velocities map
    /*
    FILE *output_file1 = fopen("../Archivos/velocities_map3D.txt", "w");
    if (output_file1 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }
    
    // Write the map data layer by layer
    for (int k = 0; k < size_map[2]; k++) {
        fprintf(output_file1, "Layer %d:\n", k);
        for (int i = 0; i < size_map[0]; i++) {
            for (int j = 0; j < size_map[1]; j++) {
                fprintf(output_file1, "%.2f ", 
                    obstacle_distance_map[j + i*size_map[1] + k*size_map[0]*size_map[1]]);
            }
            fprintf(output_file1, "\n");
        }
        fprintf(output_file1, "\n");  // Extra newline between layers
    }
    
    fclose(output_file1);
    */

    // Apply restrictions
    float* restrictions_map = restrictions3D(obstacle_distance_map, size_map, NULL);

    // Apply planner
     printf("Applying planner...\n");

     clock_t start_planner = clock();
     planners_3D(restrictions_map, size_map, objective_points, size_objective, start_points, size_start,
                planner_type, escalado_vectores, NULL, NULL, 0, 0, 0, 0);
     clock_t end_planner = clock();
     float time_planner = ((float) (end_planner - start_planner)) / CLOCKS_PER_SEC;    
     printf("Time for planner: %.3f s\n", time_planner);

     return restrictions_map;
        // Liberar memoria
     free(matriz);
     free(obstacle_distance_map);
}
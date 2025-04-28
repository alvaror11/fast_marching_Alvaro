#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include "ascension_restraint.h"
#include "common.h"
#include "FMM2.h"
#include "planner.h"


void asc_restraint_planner(float* matriz, int size_map[3], float distance_threshold, float* objective_points, 
    int size_objective[2], float* start_points, int size_start[2], float step, Trajectory3D* traj, 
    int planner_type, int escalado_vectores){
    
    int* height_map = (int*)calloc(size_map[0] * size_map[1], sizeof(int));
    if (!height_map) {
        printf("Memory allocation failed for height_map array\n");
        return;
    }
     // Create 2D occupation map
     float* occupation_map_2d = (float*)calloc(size_map[0] * size_map[1], sizeof(float));
     if (!occupation_map_2d) {
         printf("Memory allocation failed for 2D occupation map\n");
         free(height_map);
         return;
     }

    planners_3D(matriz, size_map, objective_points, size_objective, start_points, size_start, planner_type, escalado_vectores,
                height_map, occupation_map_2d);
    // Coordinates in the 3D OC.
    int start_x_3d = start_points[0];
    int start_y_3d = start_points[1];
    int objective_x_3d = objective_points[0];
    int objective_y_3d = objective_points[1];
    
    // Free 3D arrays
    free(start_points);
    free(objective_points);

    // Reallocate for 2D points
    start_points = (float *)malloc(2 * sizeof(float));
    objective_points = (float *)malloc(2 * sizeof(float));

    // Update array sizes for 2D
    size_start[0] = 2;    // dimensions now 2 instead of 3
    size_objective[0] = 2; // dimensions now 2 instead of 3

    // Assign 2D coordinates (x e y se invierten en cuando 3d -> 2D)
    start_points[0] = start_y_3d;     // x in 2D = y in 3D
    start_points[1] = start_x_3d;     // y in 2D = x in 3D
    objective_points[0] = objective_y_3d;
    objective_points[1] = objective_x_3d;

    // Redefine size_map for 2D
    int size_x_3d = size_map[0];
    int size_y_3d = size_map[1];

    int* size_map_2d = (int*)malloc(2 * sizeof(int));
    size_map_2d[0] = size_y_3d;
    size_map_2d[1] = size_x_3d;
    // El planner ya se ha aplicado, se pone a 0 para que no se vuelva a aplicar
    planner_type = 0;
    
    // Crear la trayectoria
    int initial_capacity = 100;
    Trajectory* traj_2D = malloc(sizeof(Trajectory));
    traj_2D->points = malloc(initial_capacity * sizeof(Point2D));
    traj_2D->size = 0;
    traj_2D->capacity = initial_capacity;

    FMM2_2D(matriz, size_map_2d, distance_threshold, 
            objective_points, size_objective, start_points, size_start, step, traj_2D, planner_type, escalado_vectores);
    
    }
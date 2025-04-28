#ifndef PLANNER_H
#define PLANNER_H

#include <stdbool.h>

void planners_2D(float* matriz, 
                 int* size_map, 
                 float* objective_points, 
                 int size_objective[2], 
                 float* start_points, int size_start[2], 
                 int planner_type, int escalado_vectores);

void planners_3D(float* matriz, 
                  int* size_map, 
                  float* objective_points, 
                  int size_objective[2], 
                  float* start_points, int size_start[2], 
                  int planner_type, int escalado_vectores,
                  int* height_map, float* occupation_map_2d);

#endif // PLANNER_H
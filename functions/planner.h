#ifndef PLANNER_H
#define PLANNER_H

#include <stdbool.h>

void planners_2D(double* matriz, 
                 int* size_map, 
                 double* objective_points, 
                 int size_objective[2], 
                 double* start_points, int size_start[2], 
                 int planner_type, int escalado_vectores);

#endif // PLANNER_H
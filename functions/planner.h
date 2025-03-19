#ifndef PLANNER_H
#define PLANNER_H

#include <stdbool.h>

void planners_2D(double* matriz, 
                 int* size_map, 
                 double* objective_points, 
                 int size_objective[2], 
                 double* start_points, 
                 int planner_type);

#endif // PLANNER_H
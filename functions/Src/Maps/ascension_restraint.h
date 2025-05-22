#ifndef ASCENSION_RESTRAINT_H
#define ASCENSION_RESTRAINT_H

#include "common.h"
#include "FMM2.h"

void asc_restraint_planner(float* matriz, int size_map[3], float distance_threshold, float* objective_points, 
    int size_objective[2], float* start_points, int size_start[2], float step, Trajectory3D* traj, 
    int planner_type, float escalado_vectores, int ascension_rate, int descent_rate, int flight_level, int resolution);

    
#endif 


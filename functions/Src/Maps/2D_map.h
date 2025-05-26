#ifndef TWOD_MAP_H
#define TWOD_MAP_H

float* velocities_map(float* binary_map, int* size_map, int threshold);
float* restrictions2D(float* viscosity_map, int* size_map, char* dir,
                        float* objective_points, int* size_objective, float* start_points, int size_start[2]);

#endif
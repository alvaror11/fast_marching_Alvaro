#ifndef THREED_MAP_H
#define THREED_MAP_H

float* velocities_map3D(float* binary_map, int* size_map, int threshold);
float* restrictions3D(float* viscosity_map, int* size_map, char* dir, 
                      float* objective_points, int* size_objective, float* start_points, int size_start[2]);

#endif
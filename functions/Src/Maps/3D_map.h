#ifndef THREED_MAP_H
#define THREED_MAP_H

float* velocities_map3D(float* binary_map, int* size_map, int threshold);
float* restrictions3D(float* viscosity_map, int* size_map, char* dir);

#endif
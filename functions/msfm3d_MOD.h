#ifndef MSFM2D_MOD_H
#define MSFM2D_MOD_H

double* main_msfm3D(double* F, double* source_points, double* T, int* size_map, int* size_target);
void compute_gradient_3d_discrete(double* input_matrix, double* gradient_matrix, int* size_map);
double* velocities_map3D(double* matriz, int* size_map, int distance_threshold);

#endif 
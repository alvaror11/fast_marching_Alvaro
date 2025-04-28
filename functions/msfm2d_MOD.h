#ifndef MSFM2D_MOD_H
#define MSFM2D_MOD_H

float* main_msfm(float* F, float* source_points, float* T, int* size_map, int* size_target);
float* velocities_map(float* binary_map, int* size_map, int threshold);
void compute_gradient_2d_discrete(float* input_matrix, float* gradient_matrix, int* size_map);

#endif 
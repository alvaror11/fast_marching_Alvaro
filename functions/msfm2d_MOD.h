#ifndef MSFM2D_MOD_H
#define MSFM2D_MOD_H

double* main_msfm(double* F, double* source_points, double* T, int* size_map, int* size_target);
double* velocities_map(double* binary_map, int rows, int cols, int threshold);
void compute_gradient_2d_discrete(double* input_matrix, double* gradient_matrix, int rows, int cols);


#endif 
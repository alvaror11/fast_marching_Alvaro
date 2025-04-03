#ifndef MSFM2D_MOD_H
#define MSFM2D_MOD_H

    float* main_msfm3D(float* F, float* source_points, float* T, int* size_map, int* size_target,
                        float* start_points);
    void compute_gradient_3d_discrete(float* input_matrix, float* gradient_matrix, int* size_map);
    float* velocities_map3D(float* matriz, int* size_map, int distance_threshold);

#endif 
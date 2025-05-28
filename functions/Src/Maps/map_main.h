#ifndef map_main_H
#define map_main_H

float* map_main2D(float* matriz, int* size_map, int distance_threshold,
                float* objective_points, int* size_objective, float* start_points, int size_start[2],
                int planner_type, float escalado_vectores, bool dosymedioD);

float* map_main3D(float* matriz, int* size_map, int distance_threshold,
                float* objective_points, int* size_objective, float* start_points, int size_start[2],
                int planner_type, float escalado_vectores);

#endif
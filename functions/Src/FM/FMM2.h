#ifndef FMM2_H
#define FMM2_H

#include <stdbool.h>

// Point structures
typedef struct {
    float x;
    float y;
} Point2D;

typedef struct {
    float x;
    float y;
    float z;
} Point3D;

// Trajectory structures
typedef struct {
    Point2D* points;
    int size;
    int capacity;
} Trajectory;

typedef struct {
    Point3D* points;
    int size;
    int capacity;
} Trajectory3D;

// Function declarations
void FMM2_2D(float* matriz, int* size_map, float distance_threshold, 
             float* objective_points, int size_objective[2], 
             float* start_points, int size_start[2], float step, Trajectory* traj, 
             int planner_type, int escalado_vectores);

void FMM2_3D(float* matriz, int size_map[3], float distance_threshold, float* objective_points, 
             int size_objective[2], float* start_points, int size_start[2], 
             float step, Trajectory3D* traj, 
             int planner_type, int escalado_vectores, float* occupation_map);  

// Helper functions
void addPointToTrajectory(Trajectory* traj, float x, float y);
void addPointToTrajectory3D(Trajectory3D* traj, float x, float y, float z);

#endif // FMM2_H
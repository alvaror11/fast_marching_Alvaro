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
<<<<<<< HEAD
void FMM2_2D(float* matriz, int* size_map, float distance_threshold, 
             float safety_margin, float* objective_points, int size_objective[2], 
             float* start_points, int size_start[2], float step, Trajectory* traj, 
=======
void FMM2_2D(double* matriz, int* size_map, double distance_threshold, 
             double safety_margin, double* objective_points, int size_objective[2], 
             double* start_points, int size_start[2], double step, Trajectory* traj, 
>>>>>>> 73e63430f622407039e3c29ab88c8c8644f0e79f
             int planner_type, int escalado_vectores);

void FMM2_3D(float* matriz, int* size_map, float distance_threshold,
             float* objective_points, int size_objective[2], 
             float* start_points, float step, 
             Trajectory3D* traj);  

// Helper functions
void addPointToTrajectory(Trajectory* traj, float x, float y);
void addPointToTrajectory3D(Trajectory3D* traj, float x, float y, float z);

#endif // FMM2_H
#ifndef FMM2_H
#define FMM2_H

#include <stdbool.h>

// Point structures
typedef struct {
    double x;
    double y;
} Point2D;

typedef struct {
    double x;
    double y;
    double z;
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
void FMM2_2D(double* matriz, int* size_map, double distance_threshold, 
             double safety_margin, double* objective_points, int size_objective[2], 
             double* start_points, double step, Trajectory* traj);

void FMM2_3D(double* matriz, int* size_map, double distance_threshold,
             double* objective_points, int size_objective[2], 
             double* start_points, double step, 
             Trajectory3D* traj);  

// Helper functions
void addPointToTrajectory(Trajectory* traj, double x, double y);
void addPointToTrajectory3D(Trajectory3D* traj, double x, double y, double z);

#endif // FMM2_H
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "common.h"
#include "rk4_2D_3D.h"
#include "planner.h"



double* velocities_map3D(double* matriz, int* size_map, int distance_threshold);
double* main_msfm3D(double* obstacle_distance_map, double* objective_points, double* output_T, 
                    int* size_map, int* size_objective, double* start_points);
void compute_gradient_3d_discrete(double* input_matrix, double* gradient_matrix, int* size_map);

// Estructura de los puntos de la trayectoria
typedef struct {
    double x;
    double y;
} Point2D;

typedef struct {
    double x;
    double y;
    double z;
} Point3D;

//Estructura de la trayectoria
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

void compute_2d_trajectory();
void addPointToTrajectory(Trajectory* traj, double x, double y) {
    if (traj->size >= traj->capacity) {
        traj->capacity *= 2;
        Point2D* temp = realloc(traj->points, traj->capacity * sizeof(Point2D));
        if (temp == NULL) {
            // Manejar error de memoria
            return;
        }
        traj->points = temp;
    }
    
    traj->points[traj->size].x = x;
    traj->points[traj->size].y = y;
    traj->size++;
}
void addPointToTrajectory3D(Trajectory3D* traj, double x, double y, double z) {
    if (traj->size >= traj->capacity) {
        traj->capacity *= 2;
        Point3D* temp = realloc(traj->points, traj->capacity * sizeof(Point3D));
        if (temp == NULL) {
            // Manejar error de memoria
            return;
        }
        traj->points = temp;
    }
    
    traj->points[traj->size].x = x;
    traj->points[traj->size].y = y;
    traj->points[traj->size].z = z;
    traj->size++;
}


void FMM2_2D(double* matriz, int* size_map, double distance_threshold, double safety_margin,
             double* objective_points, int size_objective[2], double* start_points, int size_start[2], double step,
            Trajectory* traj, int planner_type, int escalado_vectores){
    

    if (planner_type == 0){
        // Solo se añade un borde de obstaculos alrededor del mapa si no se ha aplicado ningun planner
        size_map[0] += 2;
        size_map[1] += 2;
        int columnas = size_map[0];
        int filas = size_map[1];
        
        objective_points[0] = objective_points[0] + 1;
        objective_points[1] = objective_points[1] + 1;
        start_points[0] = start_points[0] + 1;
        start_points[1] = start_points[1] + 1;
        double *matriz2 = (double *)malloc(filas * columnas * sizeof(double));
        for (int i = 0; i < filas; i++) {
            for (int j = 0; j < columnas; j++) {
                if (i == 0 || j == 0 || i == filas-1 || j == columnas-1) {
                    matriz2[j + i * columnas] = 1;
                } else {
                    matriz2[j + i * columnas] = matriz[j-1 + (i-1) * (columnas-2)];
                }
            }
        }

        free(matriz);
        matriz = matriz2;
    }     
    //Create velocities map
    printf("Creating velocities map...\n");
    double* obstacle_distance_map = velocities_map(matriz, size_map, distance_threshold, safety_margin);
    
    // Apply planner
    planners_2D(obstacle_distance_map, size_map, objective_points, size_objective, start_points, size_start, planner_type, escalado_vectores);
    
    // Save velocities map 
    /*
    FILE *output_file1 = fopen("./Archivos/velocities_map.txt", "w");
     if (output_file1 == NULL) {
         perror("Error al abrir el archivo de salida");
         return;
     }
     for (int i = 0; i < size_map[1]; i++) {
         for (int j = 0; j < size_map[0]; j++) {
             fprintf(output_file1, "%.2f ", obstacle_distance_map[j + i * size_map[0]]);
         }
         fprintf(output_file1, "\n");
     }
     fclose(output_file1);
     */
    
    
     
    
    //Allocate memory for output map
    double* output_T = (double *)malloc(size_map[1] * size_map[0] * sizeof(double));
    printf("Calculating times map...\n");  
    output_T = main_msfm(obstacle_distance_map, objective_points, output_T, size_map, size_objective);

    // Save times map
    /*
    FILE *output_file2 = fopen("./Archivos/times_map.txt", "w");
     if (output_file2 == NULL) {
         perror("Error al abrir el archivo de salida");
         return;
     }
 
     // Escribir los resultados del mapa de distancias en el archivo de salida
     for (int i = 0; i < size_map[1]; i++) {
         for (int j = 0; j < size_map[0]; j++) {
             fprintf(output_file2, "%.2f ", output_T[j + i * size_map[0]]);
         }
         fprintf(output_file2, "\n");
     }
     fclose(output_file2);
    */
    // Crear el gradiente para el mapa de tiempos:
    double* gradient_matrix = (double*)malloc(2 * size_map[1] * size_map[0] * sizeof(double));
    printf("Calculating gradient...\n");
    compute_gradient_2d_discrete(output_T, gradient_matrix, size_map);
    // Save gradients
    /*
    FILE *gradient_x_file = fopen("./Archivos/gradient_x.txt", "w");
    FILE *gradient_y_file = fopen("./Archivos/gradient_y.txt", "w");
 
     if (gradient_x_file == NULL || gradient_y_file == NULL) {
         perror("Error al abrir los archivos de gradiente");
         return;
     }
 
     // Write gradient components in matrix format
     for (int i = 0; i < size_map[1]; i++) {
         for (int j = 0; j < size_map[0]; j++) {
             // Write x component
             fprintf(gradient_x_file, "%.4f ", gradient_matrix[i*size_map[0] + j]);
             // Write y component
             fprintf(gradient_y_file, "%.4f ", gradient_matrix[i*size_map[0] + j + size_map[1]*size_map[0]]);
         }
         fprintf(gradient_x_file, "\n");
         fprintf(gradient_y_file, "\n");
     }
 
     fclose(gradient_x_file);
     fclose(gradient_y_file);
    */

    // Empezamos a usar el descenso del gradiente para buscar el camino
    bool finished = false;      //mientras no se llegue al punto final es false
     
    // Añadir el punto inicial
    printf("Starting gradient descend...\n");
    addPointToTrajectory(traj, start_points[0], start_points[1]);
    //Definir el pointer para el último punto de la trayectoria y el nuevo punto
    double* last_point = malloc(2 * sizeof(double));
    double* new_point = malloc(2 * sizeof(double));

    while (finished == false){
        // Obtener las coordenadas del último punto de la trayectoria
        
        last_point[0] = traj->points[traj->size - 1].x;
        last_point[1] = traj->points[traj->size - 1].y;

        // Llamar a la función mexFunction en rk4_2D con las coordenadas del último punto y el mapa de velocidades
        gradient_descend_rk4(last_point, gradient_matrix, size_map, size_objective, new_point, step);

        // Añadir el nuevo punto a la trayectoria
        addPointToTrajectory(traj, new_point[0], new_point[1]);
        // Comprobar si se ha llegado al punto objetivo
        if ((round(new_point[0]) == objective_points[0] || floor(new_point[0]) == objective_points[0]) 
            && (round(new_point[1]) == objective_points[1] || floor(new_point[1]) == objective_points[1])) {
            finished = true;
            traj->points[traj->size - 1].x = objective_points[0];
            traj->points[traj->size - 1].y = objective_points[1];
            printf("Objective reached\n");
        }
    }
    /*
    FILE *traj_file = fopen("./Archivos/trajectory.txt", "w");
     if (traj_file == NULL) {
         perror("Error al abrir el archivo de trayectoria");
         return;
     }
 
     for (int i = 0; i < traj->size; i++) {
         double traj_x = traj->points[i].x;
         double traj_y = traj->points[i].y;
         // Save to file (x y format)
         fprintf(traj_file, "%.2f %.2f\n", traj_x, traj_y);
     }
 
     fclose(traj_file);
    
    // Liberar memoria    
    free(output_T);
    free(matriz);
    free(objective_points);
    free(start_points);
    //free(obstacle_distance_map);
    free(last_point);
    */
}
void FMM2_3D(double* matriz, int size_map[3], double distance_threshold, double* objective_points, 
            int size_objective[2], double* start_points, double step, Trajectory3D* traj){
    
    //Create velocities map
    printf("Creating velocities map...\n");
    clock_t start_velocitiesMap = clock();
    double* obstacle_distance_map = velocities_map3D(matriz, size_map, distance_threshold);
    clock_t end_velocitiesMap = clock();
    double time_velocitiesMap = ((double) (end_velocitiesMap - start_velocitiesMap)) / CLOCKS_PER_SEC;
    printf("Time for velocities map: %.3f s\n", time_velocitiesMap);
    // Save velocities map
    
    FILE *output_file1 = fopen("./Archivos/velocities_map3D.txt", "w");
    if (output_file1 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }
    
    // Write the map data layer by layer
    for (int k = 0; k < size_map[2]; k++) {
        fprintf(output_file1, "Layer %d:\n", k);
        for (int i = 0; i < size_map[0]; i++) {
            for (int j = 0; j < size_map[1]; j++) {
                fprintf(output_file1, "%.2f ", 
                    obstacle_distance_map[j + i*size_map[1] + k*size_map[0]*size_map[1]]);
            }
            fprintf(output_file1, "\n");
        }
        fprintf(output_file1, "\n");  // Extra newline between layers
    }
    
    fclose(output_file1);
    
    //Allocate memory for output map
    double* output_T = (double *)malloc(size_map[0] * size_map[1] * size_map[2] * sizeof(double));
    printf("Calculating times map...\n");
    clock_t start_timesMap = clock();
    output_T = main_msfm3D(obstacle_distance_map, objective_points, output_T, size_map, size_objective, start_points);
    clock_t end_timesMap = clock();
    double time_timesMap = ((double) (end_timesMap - start_timesMap)) / CLOCKS_PER_SEC;
    printf("Time for fast marching: %.3f s\n", time_timesMap);
    // Save times map
    
   FILE *output_file2 = fopen("./Archivos/times_map3D.txt", "w");
    if (output_file2 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }
    
    // Write dimensions in first line
    fprintf(output_file2, "%d %d %d\n", size_map[0], size_map[1], size_map[2]);
    
    // Write times map layer by layer
    for (int k = 0; k < size_map[2]; k++) {
        fprintf(output_file2, "Layer %d:\n", k);
        for (int i = 0; i < size_map[1]; i++) {
            for (int j = 0; j < size_map[0]; j++) {
                fprintf(output_file2, "%.2f ", 
                    output_T[j + i*size_map[0] + k*size_map[0]*size_map[1]]);
            }
            fprintf(output_file2, "\n");
        }
        fprintf(output_file2, "\n");
    }
    fclose(output_file2);
    
    // Crear el gradiente para el mapa de tiempos:
    double* gradient_matrix = (double*)malloc(3 * size_map[0] * size_map[1] * size_map[2] * sizeof(double));
    printf("Calculating gradient...\n");
    clock_t start_gradient = clock();
    compute_gradient_3d_discrete(output_T, gradient_matrix, size_map);
    clock_t end_gradient = clock();
    double time_gradient = ((double) (end_gradient - start_gradient)) / CLOCKS_PER_SEC;
    printf("Time for gradient: %.3f s\n", time_gradient);

     // Save gradients
     
     FILE *gradient_x_file = fopen("./Archivos/gradient3D_x.txt", "w");
     FILE *gradient_y_file = fopen("./Archivos/gradient3D_y.txt", "w");
     FILE *gradient_z_file = fopen("./Archivos/gradient3D_z.txt", "w");
     
     if (gradient_x_file == NULL || gradient_y_file == NULL || gradient_z_file == NULL) {
         perror("Error al abrir los archivos de gradiente");
         return;
     }
 
     // Write dimensions in first line of each file
     fprintf(gradient_x_file, "%d %d %d\n", size_map[0], size_map[1], size_map[2]);
     fprintf(gradient_y_file, "%d %d %d\n", size_map[0], size_map[1], size_map[2]);
     fprintf(gradient_z_file, "%d %d %d\n", size_map[0], size_map[1], size_map[2]);
 
     // Write gradient components layer by layer
     for (int k = 0; k < size_map[2]; k++) {
         fprintf(gradient_x_file, "Layer %d:\n", k);
         fprintf(gradient_y_file, "Layer %d:\n", k);
         fprintf(gradient_z_file, "Layer %d:\n", k);
         
         for (int i = 0; i < size_map[0]; i++) {
             for (int j = 0; j < size_map[1]; j++) {
                 int idx = j + i*size_map[1] + k*size_map[0]*size_map[1];
                 // Write x component
                 fprintf(gradient_x_file, "%.4f ", gradient_matrix[idx]);
                 // Write y component
                 fprintf(gradient_y_file, "%.4f ", gradient_matrix[idx + size_map[0]*size_map[1]*size_map[2]]);
                 // Write z component
                 fprintf(gradient_z_file, "%.4f ", gradient_matrix[idx + 2*size_map[0]*size_map[1]*size_map[2]]);
             }
             fprintf(gradient_x_file, "\n");
             fprintf(gradient_y_file, "\n");
             fprintf(gradient_z_file, "\n");
         }
         fprintf(gradient_x_file, "\n");
         fprintf(gradient_y_file, "\n");
         fprintf(gradient_z_file, "\n");
     }
 
     fclose(gradient_x_file);
     fclose(gradient_y_file);
     fclose(gradient_z_file);
        
    // Empezamos a usar el descenso del gradiente para buscar el camino
    bool finished = false;      //mientras no se llegue al punto final es false

    // Añadir el punto inicial
    addPointToTrajectory3D(traj, start_points[0], start_points[1], start_points[2]);
    printf("Starting gradient descend...\n");
    clock_t start_gradient_descend = clock();
    //Definir el pointer para el último punto de la trayectoria y el nuevo punto
    double* last_point = malloc(3 * sizeof(double));
    double* new_point = malloc(3 * sizeof(double));
    while (finished == false){
        // Obtener las coordenadas del último punto de la trayectoria
        
        last_point[0] = traj->points[traj->size - 1].x;
        last_point[1] = traj->points[traj->size - 1].y;
        last_point[2] = traj->points[traj->size - 1].z;
        
        // Llamar a la función mexFunction en rk4_2D con las coordenadas del último punto y el mapa de velocidades
        gradient_descend_rk4(last_point, gradient_matrix, size_map, size_objective, new_point, step);

        // Añadir el nuevo punto a la trayectoria
        addPointToTrajectory3D(traj, new_point[0], new_point[1], new_point[2]);
        

        // Comprobar si se ha llegado al punto objetivo
        if ((round(new_point[0]) == objective_points[0] || floor(new_point[0]) == objective_points[0]) 
            && (round(new_point[1]) == objective_points[1] || floor(new_point[1]) == objective_points[1])
            &&(round(new_point[2]) == objective_points[2] || floor(new_point[2]) == objective_points[2])) {
            finished = true;
            traj->points[traj->size - 1].x = objective_points[0];
            traj->points[traj->size - 1].y = objective_points[1];
            traj->points[traj->size - 1].z = objective_points[2];
        }
    }
    clock_t end_gradient_descend = clock();
    double time_gradient_descend = ((double) (end_gradient_descend - start_gradient_descend)) / CLOCKS_PER_SEC;
    printf("Time for gradient descend: %.3f s\n", time_gradient_descend);
    // Save trajectory to file
    
    FILE *traj_file = fopen("./Archivos/trajectory3D.txt", "w");
    if (traj_file == NULL) {
        perror("Error al abrir el archivo de trayectoria");
        return;
    }

    // Write header with number of points and dimensions
    fprintf(traj_file, "%d %d\n", traj->size, 3);  // 3 dimensions

    // Write trajectory points
    for (int i = 0; i < traj->size; i++) {
        fprintf(traj_file, "%.2f %.2f %.2f\n", 
            traj->points[i].x,
            traj->points[i].y,
            traj->points[i].z);
        
    }

    fclose(traj_file);
    
    
    // CHECK TRAJECTORY FOR COLLISIONS
    printf("\nChecking trajectory for obstacle collisions...\n");
    for (int i = 0; i < traj->size; i++) {
        int x = (int)round(traj->points[i].x) - 1;
        int y = (int)round(traj->points[i].y) - 1;
        int z = (int)round(traj->points[i].z) - 1;
        
        
        // Check if point is in obstacle (matriz has 1s for obstacles)
        if (matriz[y-1 + (x-1)*size_map[1] + (z-1)*size_map[0]*size_map[1]] == 1) {
            printf("Warning: Point %d (%.2f, %.2f, %.2f) intersects with obstacle\n", 
                i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
        }
    }
    

    // Liberar memoria    
    free(output_T);
    free(matriz);
    free(objective_points);
    free(start_points);
    free(obstacle_distance_map);
    free(last_point);
}

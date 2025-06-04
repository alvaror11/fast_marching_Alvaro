#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "../common.h"
#include "rk4_2D_3D.h"
#include "planner.h"



float* velocities_map3D(float* matriz, int* size_map, int distance_threshold);
float* main_msfm3D(float* obstacle_distance_map, float* objective_points, float* output_T, 
                    int* size_map, int* size_objective, float* start_points);
void compute_gradient_3d_discrete(float* input_matrix, float* gradient_matrix, int* size_map);

// Estructura de los puntos de la trayectoria
typedef struct {
    float x;
    float y;
} Point2D;

typedef struct {
    float x;
    float y;
    float z;
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
void addPointToTrajectory(Trajectory* traj, float x, float y) {
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
void addPointToTrajectory3D(Trajectory3D* traj, float x, float y, float z) {
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


void FMM2_2D(float* restrictions_map, int* size_map, float distance_threshold,
             float* objective_points, int size_objective[2], float* start_points, int size_start[2], float step,
            Trajectory* traj, int planner_type, int escalado_vectores){
    

    //Allocate memory for output map
     float* output_T = (float *)malloc(size_map[1] * size_map[0] * sizeof(float));
    printf("Calculating times map...\n"); 
    output_T = main_msfm(restrictions_map, objective_points, output_T, size_map, size_objective);
    
    // Save times map
    FILE *output_file2 = fopen("../Archivos/times_map.txt", "w");
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
    
    // Crear el gradiente para el mapa de tiempos:
    float* gradient_matrix = (float*)malloc(2 * size_map[1] * size_map[0] * sizeof(float));
    printf("Calculating gradient...\n");
    compute_gradient_2d_discrete(output_T, gradient_matrix, size_map);
    
    // Save gradients
    FILE *gradient_x_file = fopen("../Archivos/gradient_x.txt", "w");
    FILE *gradient_y_file = fopen("../Archivos/gradient_y.txt", "w");
 
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
    

    //Descenso del gradiente 
    bool finished = false;      //mientras no se llegue al punto final es false
     
    // ATENCION, los puntos se guardan  empezando con i = 1, y ademas normalmente se considera que estan encapsulados
    // en el futuro habra que mirar de que forma se guardan las coordenadas y se debera modificar
    // Añadir el punto inicial
    printf("Starting gradient descend...\n");
    addPointToTrajectory(traj, start_points[0], start_points[1]);
    //Definir el pointer para el último punto de la trayectoria y el nuevo punto
    float* last_point = malloc(2 * sizeof(float));
    float* new_point = malloc(2 * sizeof(float));

    int iteraciones = 0;
    while (finished == false){
        // Obtener las coordenadas del último punto de la trayectoria
        
        last_point[0] = traj->points[traj->size - 1].x;
        last_point[1] = traj->points[traj->size - 1].y;

        // Llamar a la función mexFunction en rk4_2D con las coordenadas del último punto y el mapa de velocidades
        gradient_descend_rk4(last_point, gradient_matrix, size_map, size_objective, new_point, step);

        if (isnan(new_point[0]) || isnan(new_point[1])) {
            iteraciones ++;
        }
        if (iteraciones > 5) {
            break;
        }
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
    if (iteraciones > 5){
        printf("Could not find a path\n");
        // Clear the trajectory
        traj->size = 0;  // Set size to 0
        free(traj->points);  // Free allocated memory
        traj->points = NULL;  // Set pointer to NULL
        traj->capacity = 0;  // Reset capacity
        
        // Free other allocated memory before returning
        free(output_T);
        free(gradient_matrix);
        free(last_point);
        free(new_point);
        return;
    }
    
    FILE *traj_file = fopen("../Archivos/trajectory.txt", "w");
     if (traj_file == NULL) {
         perror("Error al abrir el archivo de trayectoria");
         return;
     }
     
     for (int i = 0; i < traj->size; i++) {
         float traj_x = traj->points[i].x;
         float traj_y = traj->points[i].y;
         // Save to file (x y format)
         fprintf(traj_file, "%.2f %.2f\n", traj_x, traj_y);
     }
 
     fclose(traj_file);
    
    // Liberar memoria    
    free(output_T);
    free(last_point);
    free(new_point);
    
}
void FMM2_3D(float* restrictions_map, int size_map[3], float distance_threshold, float* objective_points, 
            int size_objective[2], float* start_points, int size_start[2], float step, Trajectory3D* traj, 
            int planner_type, int escalado_vectores, float* occupation_map){
    
    
    //Allocate memory for output map
    float* output_T = (float *)malloc(size_map[0] * size_map[1] * size_map[2] * sizeof(float));
    printf("Calculating times map...\n");
    clock_t start_timesMap = clock();
    output_T = main_msfm3D(restrictions_map, objective_points, output_T, size_map, size_objective, start_points);
    clock_t end_timesMap = clock();
    float time_timesMap = ((float) (end_timesMap - start_timesMap)) / CLOCKS_PER_SEC;
    printf("Time for fast marching: %.3f s\n", time_timesMap);
    // Save times map
    
   FILE *output_file2 = fopen("../Archivos/times_map3D.txt", "w");
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
                    output_T[j + i*size_map[1] + k*size_map[0]*size_map[1]]);
            }
            fprintf(output_file2, "\n");
        }
        fprintf(output_file2, "\n");
    }
    fclose(output_file2);
    
    // Crear el gradiente para el mapa de tiempos:
    float* gradient_matrix = (float*)malloc(3 * size_map[0] * size_map[1] * size_map[2] * sizeof(float));
    printf("Calculating gradient...\n");
    clock_t start_gradient = clock();
    compute_gradient_3d_discrete(output_T, gradient_matrix, size_map);
    clock_t end_gradient = clock();
    float time_gradient = ((float) (end_gradient - start_gradient)) / CLOCKS_PER_SEC;
    printf("Time for gradient: %.3f s\n", time_gradient);

     // Save gradients
     
     FILE *gradient_x_file = fopen("../Archivos/gradient3D_x.txt", "w");
     FILE *gradient_y_file = fopen("../Archivos/gradient3D_y.txt", "w");
     FILE *gradient_z_file = fopen("../Archivos/gradient3D_z.txt", "w");
     
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
        
    // Descenso del gradiente
    bool finished = false;      //mientras no se llegue al punto final es false

    // ATENCION, los puntos se guardan  empezando con i = 1, y ademas normalmente se considera que estan encapsulados
    // en el futuro habra que mirar de que forma se guardan las coordenadas y se debera modificar
    // Añadir el punto inicial
    addPointToTrajectory3D(traj, start_points[0], start_points[1], start_points[2]);
    printf("Starting gradient descend...\n");
    clock_t start_gradient_descend = clock();
    //Definir el pointer para el último punto de la trayectoria y el nuevo punto
    float* last_point = malloc(3 * sizeof(float));
    float* new_point = malloc(3 * sizeof(float));
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
    float time_gradient_descend = ((float) (end_gradient_descend - start_gradient_descend)) / CLOCKS_PER_SEC;
    printf("Time for gradient descend: %.3f s\n", time_gradient_descend);
    // Save trajectory to file
    
    FILE *traj_file = fopen("../Archivos/trajectory3D.txt", "w");
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
    
    

    // Liberar memoria    
    free(output_T);
    free(gradient_matrix);
    free(last_point);
}

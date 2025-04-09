#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "common.h"
#include "rk4_2D_3D.h"
#include "FMM2.h"
#include "map.h"

#ifdef WINDOWS
#include <windows.h>
#include <psapi.h>
#endif


void main() {
    
    // Choose dimensions of the trayectory
     int dimensions_prob = 2;// Removed redefinition of 'dimensions'

    if (dimensions_prob == 3){
        clock_t start = clock();
        // Coord X = ancho, Y = largo, Z = alto

       const char* mapfile = "./Mapas/MAP_3_100_100_100.txt";
        int ancho, largo, alto;
        if (sscanf(mapfile, "./Mapas/MAP_%*d_%d_%d_%d.txt", &ancho, &largo, &alto) != 3) {
            printf("Error: Could not extract dimensions from filename. Using defaults.\n");
            ancho = 50;
            largo = 50;
            alto = 50;
        }
        
        FILE *file = fopen(mapfile, "r");
        if (file == NULL) {
            perror("Error al abrir el archivo");
            return;
        }
        //int ancho = 50, largo = 50, alto = 50 ; 
        int *size_map = (int *)malloc(3 * sizeof(int));
        size_map[0] = ancho;
        size_map[1] = largo;
        size_map[2] = alto;
        
        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        int size_start[2] = {3, num_start_points};
        float *start_points = (float *)malloc(num_start_points * 3 * sizeof(float));;
        start_points[0] = 10;    // x coordinate
        start_points[1] = 10;   // y coordinate
        start_points[2] = 10;   // z coordinate

        // Define las coordenadas objetivo
        int num_points = 1;
        // Removed redefinition of 'dimensions'
        int size_objective[2] = {3,1};
        float *objective_points  = (float *)malloc(num_points * 3 * sizeof(float));;
        objective_points[0] = 86;   // x coordinate
        objective_points[1] = 80;    // y coordinate
        objective_points[2] = 92;    // z coordinate

        // PARAMETROS PARA LOS PLANNER
        int planner_type = 1;           //tipo de planner a usar
        int escalado_vectores = 5;      //valor para escalar los vectores del planner 2
        
        // Define el umbral de distancia para la matriz de velocidades
        float distance_threshold = 4.0;

        // Define el tamaño del paso
        float step = 0.5;

        float *matriz = (float *)malloc(ancho * largo * alto* sizeof(float));
        //printf("\nReading 3D map with dimensions: %d x %d x %d\n", ancho, largo, alto);

        // Leer los datos y asignarlos a la matriz
        for (int k = 0; k < alto; k++) {
            for (int i = 0; i < ancho; i++) {
                for (int j = 0; j < largo; j++) {
                    int valor;
                    fscanf(file, "%d", &valor);
                    int index = j + i*largo + k*ancho*largo;
                    matriz[index] = (float)valor;
                }
            }
            char newline[2];
            fgets(newline, sizeof(newline), file);
        }


        fclose(file);

        // Check that initial and final points are not inside an obstacle
        if ((matriz[(int)start_points[0] - 1 + ((int)start_points[1] -1)*largo + ((int)start_points[2] - 1)*ancho*largo] == 1)||
            (matriz[(int)objective_points[0] - 1 + ((int)objective_points[1] - 1)*largo + ((int)objective_points[2] - 1)*ancho*largo] == 1)) {
            printf("Error: Initial or objective point is inside an obstacle\n");
            return;
        }
        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] || objective_points[2] > size_map[2] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1] || start_points[2] > size_map[2]){ 
            printf("Error: Initial or objective point is outside the map\n");
            return;
        }


        // In your compute_3d_trajectory function:
        int initial_capacity = 100;
        Trajectory3D* traj = malloc(sizeof(Trajectory3D));
        traj->points = malloc(initial_capacity * sizeof(Point3D));  // Initial capacity
        traj->size = 0;
        traj->capacity = initial_capacity;

        // Call th FMM2 function
        
        FMM2_3D(matriz, size_map, distance_threshold, 
            objective_points, size_objective, start_points, size_start,
             step, traj, planner_type, escalado_vectores);
        clock_t end = clock();
        float cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
        // Save trajectory to file
        /*
        FILE* results_file = fopen("./Archivos/trajectory_results.txt", "w");
        if (results_file == NULL) {
            perror("Error opening results file");
            return;
        }

        fprintf(results_file, "=== Map Characteristics ===\n");
        fprintf(results_file, "3D (%d x %d x %d)\n\n", size_map[0], size_map[1], size_map[2]);

        fprintf(results_file, "=== Computation Parameters ===\n");
        fprintf(results_file, "Computation Time = %.3f s\n", cpu_time_used);


        fprintf(results_file, "=== Trajectory Parameters ===\n");
        fprintf(results_file, "Start point: (%.2f, %.2f. %.2f)\n", start_points[0], start_points[1], start_points[2]);
        fprintf(results_file, "End point: (%.2f, %.2f, %.2f)\n", objective_points[0], objective_points[1], objective_points[2]);
        fprintf(results_file, "Distance threshold: %.2f\n\n", distance_threshold);

        fprintf(results_file, "=== Trajectory Points ===\n");
        for (int i = 0; i < traj->size; i++) {
            fprintf(results_file, "Point %d: (%.2f, %.2f, %.2f)\n", i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
        }

        fclose(results_file);
        */
        printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);        
    }
    else if (dimensions_prob == 2){
        clock_t start = clock();
        /*
        // Define las dimensiones de la matriz
        const char* mapfile = "./Mapas/MAP_2_50_50.txt";
        int filas, columnas;
        if (sscanf(mapfile, "./Mapas/MAP_%*d_%d_%d.txt", &filas, &columnas) != 2) {
            printf("Error: Could not extract dimensions from filename. Using defaults.\n");
            filas = 50;
            columnas = 50;
        }
        
        FILE *file = fopen(mapfile, "r");
        if (file == NULL) {
            perror("Error al abrir el archivo");
            return;
        }
            */
        int columnas = MAP_COLS;
        int filas = MAP_ROWS;
        int *size_map = (int *)malloc(2 * sizeof(int));
        size_map[0] = columnas;
        size_map[1] = filas;

        // Define las coordenadas objetivo
        int num_points = 1;
        int dimensions = 2;
        int size_objective[2] = {dimensions,num_points};
        float *objective_points  = (float *)malloc(num_points * 2 * sizeof(float));;
        objective_points[0] = 38;  // x coordinate
        objective_points[1] = 10;  // y coordinate

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        int size_start[2] = {dimensions,num_start_points};
        float *start_points = (float *)malloc(num_start_points * 2 * sizeof(float));;
        start_points[0] = 15;  // x coordinate
        start_points[1] = 30;  // y coordinate
        
        // PARSAMETROS DE LOS PLANNER
        int planner_type = 0;       // tipo de planner a usar
        int escalado_vectores = 6; // valor para escalar los vectores del planner 2

        // Define el umbral de distancia para la matriz de velocidades
        float distance_threshold = 4;
        float safety_margin = 2.5;  // por ahora no se usa

        // Define el tamaño del paso para el descenso del gradiente
        float step = 0.5;

        float *matriz = (float *)malloc((int)size_map[1] * (int)size_map[0] * sizeof(float));
        memcpy(matriz, MAP_DATA, MAP_ROWS * MAP_COLS * sizeof(float));

        /*
        // Leer los datos y asignarlos a la matriz
        for (int i = 0; i < filas; i++) {
            for (int j = 0; j < columnas; j++) {
                int valor;
                fscanf(file, "%d", &valor);
                matriz[j + i * columnas] = (float)valor;
                //printf("%.1f\n",  matriz[i + j * columnas]);
            }
        }

        fclose(file);
        */
        // Check that initial and final points are not inside an obstacle
        if ((matriz[(int)start_points[0] - 1 + ((int)start_points[1]-1)*columnas] == 1)||
            (matriz[(int)objective_points[0] - 1 + ((int)objective_points[1]-1)*columnas ] == 1)) {
            printf("Error: Initial or objective point is inside an obstacle\n");
            return;
        }
        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1]){ 
            printf("Error: Initial or objective point is outside the map\n");
            return;
        }
        
        // Crear la trayectoria
        int initial_capacity = 10;
        Trajectory* traj = malloc(sizeof(Trajectory));
        traj->points = malloc(initial_capacity * sizeof(Point2D));
        traj->size = 0;
        traj->capacity = initial_capacity;
        FMM2_2D(matriz, size_map, distance_threshold, safety_margin, 
                objective_points, size_objective, start_points, size_start, step, traj, planner_type, escalado_vectores);
        clock_t end = clock();
        float cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;

        // Save trajectory to file
        FILE* results_file = fopen("./Archivos/trajectory_results.txt", "w");
        if (results_file == NULL) {
            perror("Error opening results file");
            return;
        }

        fprintf(results_file, "=== Map Characteristics ===\n");
        fprintf(results_file, "2D (%d x %d)\n\n", size_map[0], size_map[1]);

        fprintf(results_file, "=== Computation Parameters ===\n");
        fprintf(results_file, "Computation Time = %.3f s\n", cpu_time_used);


        fprintf(results_file, "=== Trajectory Parameters ===\n");
        fprintf(results_file, "Start point: (%.2f, %.2f)\n", start_points[0], start_points[1]);
        fprintf(results_file, "End point: (%.2f, %.2f)\n", objective_points[0], objective_points[1]);
        fprintf(results_file, "Distance threshold: %.2f\n\n", distance_threshold);

        fprintf(results_file, "=== Trajectory Points ===\n");
        for (int i = 0; i < traj->size; i++) {
            fprintf(results_file, "Point %d: (%.2f, %.2f)\n", i, traj->points[i].x, traj->points[i].y);
        }

        fclose(results_file);


        printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);
    }
    else{
        printf("Invalid number of dimensions\n");
    }
}
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "common.h"
#include "rk4_2D_3D.h"
#include "planner.h"
#include "FMM2.h"




void main() {
    
    // Choose dimensions of the trayectory
     int dimensions_prob = 3;// Removed redefinition of 'dimensions'

    if (dimensions_prob == 3){
        clock_t start = clock();
        // Coord X = ancho, Y = largo, Z = alto
        int ancho = 25, largo = 25, alto = 25 ; 
        int *size_map = (int *)malloc(3 * sizeof(int));
        size_map[0] = ancho;
        size_map[1] = largo;
        size_map[2] = alto;
        // Define las coordenadas objetivo
        int num_points = 1;
        // Removed redefinition of 'dimensions'
        int size_objective[2] = {3,1};
        double *objective_points  = (double *)malloc(num_points * 3 * sizeof(double));;
        objective_points[0] = 22;   // x coordinate
        objective_points[1] = 22;    // y coordinate
        objective_points[2] = 22;    // z coordinate

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        int size_start[2] = {3, num_start_points};
        double *start_points = (double *)malloc(num_start_points * 3 * sizeof(double));;
        start_points[0] = 5;    // x coordinate
        start_points[1] = 5;   // y coordinate
        start_points[2] = 5;   // z coordinate

        // PARAMETROS PARA LOS PLANNER
        int planner_type = 0;           //tipo de planner a usar
        int escalado_vectores = 5;      //valor para escalar los vectores del planner 2
        
        // Define el umbral de distancia para la matriz de velocidades
        double distance_threshold = 4.0;

        // Define el tamaño del paso
        double step = 0.5;

        FILE *file = fopen("./Mapas/MAP_3_100_100_100.txt", "r");
        if (file == NULL) {
            perror("Error al abrir el archivo");
            return;
        }
        
        double *matriz = (double *)malloc(ancho * largo * alto* sizeof(double));
        //printf("\nReading 3D map with dimensions: %d x %d x %d\n", ancho, largo, alto);

        // Leer los datos y asignarlos a la matriz
        for (int k = 0; k < alto; k++) {
            for (int i = 0; i < ancho; i++) {
                for (int j = 0; j < largo; j++) {
                    int valor;
                    fscanf(file, "%d", &valor);
                    int index = j + i*largo + k*ancho*largo;
                    matriz[index] = (double)valor;
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

        // Aplicar el planner
        clock_t start_planner = clock();
        planners_3D(matriz, size_map, objective_points, size_objective, start_points, size_start, planner_type, escalado_vectores);
        clock_t end_planner = clock();
        double time_planner = ((double) (end_planner - start_planner)) / CLOCKS_PER_SEC;    
        printf("Time for planner: %.3f s\n", time_planner);

        // Adding a layer of obstacles surrounding the map
        if (planner_type == 0){
            ancho = ancho+2;
            largo = largo+2;
            alto = alto+2;
            size_map[0] = ancho;
            size_map[1] = largo;
            size_map[2] = alto;
            objective_points[0] = objective_points[0] + 1;
            objective_points[1] = objective_points[1] + 1;
            objective_points[2] = objective_points[2] + 1;
            start_points[0] = start_points[0] + 1;
            start_points[1] = start_points[1] + 1;
            start_points[2] = start_points[2] + 1;

            double *matriz2 = (double *)malloc(ancho * largo * alto * sizeof(double));
            //printf("\nCreating surrounded 3D map with dimensions: %d x %d x %d\n", ancho, largo, alto);

            // Fill the new matrix with surrounding obstacles
            for (int k = 0; k < alto; k++) {
                for (int i = 0; i < ancho; i++) {
                    for (int j = 0; j < largo; j++) {
                        // Check if current position is on any face of the cube
                        if (i == 0 || i == ancho-1 || j == 0 || j == largo-1 || k == 0 || k == alto-1) {
                            matriz2[j + i*largo + k*ancho*largo] = 1;  // Set obstacle
                        } else {
                            // Copy original map data
                            matriz2[j + i*largo + k*ancho*largo] = 
                                matriz[(j-1) + (i-1)*(largo-2) + (k-1)*(ancho-2)*(largo-2)];
                        }
                    }
                }
            }

            free(matriz);
            matriz = matriz2;
        }
        

        // In your compute_3d_trajectory function:
        int initial_capacity = 100;
        Trajectory3D* traj = malloc(sizeof(Trajectory3D));
        traj->points = malloc(initial_capacity * sizeof(Point3D));  // Initial capacity
        traj->size = 0;
        traj->capacity = initial_capacity;

        // Call th FMM2 function
        
        FMM2_3D(matriz, size_map, distance_threshold, 
            objective_points, size_objective, start_points, step, traj);
        clock_t end = clock();
        double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
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
        // Define las dimensiones de la matriz
        int filas = 25, columnas = 25; 
        int *size_map = (int *)malloc(2 * sizeof(int));
        size_map[0] = columnas;
        size_map[1] = filas;

        // Define las coordenadas objetivo
        int num_points = 1;
        int dimensions = 2;
        int size_objective[2] = {dimensions,num_points};
        double *objective_points  = (double *)malloc(num_points * 2 * sizeof(double));;
        objective_points[0] = 5;  // x coordinate
        objective_points[1] = 5;  // y coordinate

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        int size_start[2] = {dimensions,num_start_points};
        double *start_points = (double *)malloc(num_start_points * 2 * sizeof(double));;
        start_points[0] = 20;  // x coordinate
        start_points[1] = 22;  // y coordinate
        
        // PARSAMETROS DE LOS PLANNER
        int planner_type = 0;       // tipo de planner a usar
        int escalado_vectores = 6; // valor para escalar los vectores del planner 2

        // Define el umbral de distancia para la matriz de velocidades
        double distance_threshold = 4;
        double safety_margin = 2.5;  // por ahora no se usa

        // Define el tamaño del paso para el descenso del gradiente
        double step = 0.5;

        FILE *file = fopen("./Mapas/MAP_1_25_25.txt", "r");
        if (file == NULL) {
            perror("Error al abrir el archivo");
            return;
        }

        double *matriz = (double *)malloc((int)size_map[1] * (int)size_map[0] * sizeof(double));
        // Leer los datos y asignarlos a la matriz
        for (int i = 0; i < filas; i++) {
            for (int j = 0; j < columnas; j++) {
                int valor;
                fscanf(file, "%d", &valor);
                //printf("%d", valor);
                // Invertir valores (0->1 y 1->0) y guardar en el formato esperado
                matriz[j + i * columnas] = (double)valor;
                //printf("%.1f\n",  matriz[i + j * columnas]);
            }
        }

        fclose(file);
        
        // Check that initial and final points are not inside an obstacle
        if ((matriz[(int)start_points[1] - 1 + ((int)start_points[0]-1)*columnas] == 1)||
            (matriz[(int)objective_points[1] - 1 + ((int)objective_points[0]-1)*columnas ] == 1)) {
            printf("Error: Initial or objective point is inside an obstacle\n");
            return;
        }
        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1]){ 
            printf("Error: Initial or objective point is outside the map\n");
            return;
        }
        // LLamar al planner
        planners_2D(matriz, size_map, objective_points, size_objective, start_points, size_start, planner_type, escalado_vectores);

        if (planner_type == 0){
            // Solo se añade un borde de obstaculos alrededor del mapa si no se ha aplicado ningun planner
            columnas = columnas+2;
            filas = filas+2;
            size_map[0] = columnas;
            size_map[1] = filas;
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
        
        // Crear la trayectoria
        int initial_capacity = 10;
        Trajectory* traj = malloc(sizeof(Trajectory));
        traj->points = malloc(initial_capacity * sizeof(Point2D));
        traj->size = 0;
        traj->capacity = initial_capacity;
        FMM2_2D(matriz, size_map, distance_threshold, safety_margin, 
                objective_points, size_objective, start_points, step, traj);
        clock_t end = clock();
        double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;

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
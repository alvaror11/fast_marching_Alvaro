#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include "Maps/map_main.h"
#include "common.h"
#include "FM/FMM2.h"
#include "Maps/ascension_restraint.h"


#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "rk4_2D_3D.h"


#ifdef WINDOWS
#include <windows.h>
#include <psapi.h>
#endif


void main() {
    
    // Choose dimensions of the trayectory
     int dimensions_prob = 3;// Removed redefinition of 'dimensions'

    if (dimensions_prob == 3){
        clock_t start = clock();
        // Coord X = ancho, Y = largo, Z = alto

        //const char* mapfile = "./Mapas/MAP_3_100_100_100.txt";         
        const char* mapfile = "../Mapas/Mapa_50x50.csv"; 
        //Procesar el mapa
        int* size_map = (int *)malloc(3 * sizeof(int));

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        int size_start[2] = {3, num_start_points};
        float *start_points = (float *)malloc(num_start_points * 3 * sizeof(float));;
        start_points[0] = 10;    // x coordinate
        start_points[1] = 15;   // y coordinate
        start_points[2] = 5;   // z coordinate

        // Define las coordenadas objetivo
        int num_points = 1;
        // Removed redefinition of 'dimensions'
        int size_objective[2] = {3,1};
        float *objective_points  = (float *)malloc(num_points * 3 * sizeof(float));;
        objective_points[0] = 30;   // x coordinate
        objective_points[1] = 35;    // y coordinate
        objective_points[2] = 10;     // z coordinate

        // PARAMETROS PARA LOS PLANNER
        int planner_type = 2;
        int escalado_vectores = 5;      //valor para escalar los vectores del planner 1
        int ascension_rate = 1;         
        int descent_rate = 1;           
        int flight_level =75;          // Altura de vuelo en metros
        int resolution = 5;             // Resolution in meters per cell (1 cell = resolution meters)
        
        // Define el umbral de distancia para la matriz de velocidades
        float distance_threshold = 6.0;

        // Define el tamaño del paso
        float step = 0.5;

        float *occupation_map = process_map_file((char*)mapfile, size_map, dimensions_prob);
        int ancho = size_map[0];
        int largo = size_map[1];
        int alto = size_map[2];

        // Check that initial and final points are not inside an obstacle
        if ((occupation_map[(int)start_points[1] - 1 + ((int)start_points[0] -1)*largo + ((int)start_points[2] - 1)*ancho*largo] == 1)) {
            printf("Error: Initial point is inside an obstacle\n");
            return;
        }
        if((occupation_map[(int)objective_points[1] - 1 + ((int)objective_points[0] - 1)*largo + ((int)objective_points[2] - 1)*ancho*largo] == 1)){
            printf("Error: Objective point is inside an obstacle\n");
            return;
        }
        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] || objective_points[2] > size_map[2] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1] || start_points[2] > size_map[2]){ 
            printf("Error: Initial or objective point is outside the map\n");
            return;
        }
        
        FILE* map_file = fopen("../Archivos/occupation_map.txt", "w");
        if (map_file == NULL) {
            printf("Error: Could not create occupation map file\n");
            return;
        }
        fprintf(map_file, "%d %d %d\n", size_map[0], size_map[1], size_map[2]);
        // Write map data layer by layer
        for (int z = 0; z < size_map[2]; z++) {
            fprintf(map_file, "Layer %d:\n", z);
            for (int x = 0; x < size_map[0]; x++) {
                for (int y = 0; y < size_map[1]; y++) {
                    fprintf(map_file, "%.0f ", occupation_map[y + x*size_map[1] + z*size_map[0]*size_map[1]]);
                }
                fprintf(map_file, "\n");
            }
            fprintf(map_file, "\n");
        }

        fclose(map_file);





        // Check the planner type to call one function or another
        if (planner_type == 2){
            // If the planner is 2, we need to call the ascension restraint function
            // Crear la trayectoria
            int initial_capacity = 100;
            Trajectory3D* traj = malloc(sizeof(Trajectory3D));
            traj->points = malloc(initial_capacity * sizeof(Point3D));  // Initial capacity
            traj->size = 0;
            traj->capacity = initial_capacity;
            if (traj == NULL) {
                printf("Error: Memory allocation failed for 3D trajectory.\n");
                return;
            }
            asc_restraint_planner(occupation_map, size_map, distance_threshold, 
                objective_points, size_objective, start_points, size_start,
                step, traj, planner_type, escalado_vectores, ascension_rate, 
                descent_rate, flight_level, resolution);

            clock_t end = clock();
            float cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
            printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);
            
            // Check if a trajectory was generated
            if (traj->size == 0) {
                printf("Error: No trajectory generated\n");
                free(traj->points);
                free(traj);
                free(occupation_map);
                free(size_map);
                return;
            }

            printf("\nChecking trajectory for obstacle collisions...\n");
            for (int i = 0; i < traj->size; i++) {
                int x = (int)round(traj->points[i].x) - 1;
                int y = (int)round(traj->points[i].y) - 1;
                int z = (int)round(traj->points[i].z) - 1;
                
                // Check if point is in obstacle (occupation_map has 1s for obstacles)
                if (occupation_map[y + (x)*size_map[1] + (z)*size_map[0]*size_map[1]] == 1) {
                    printf("Warning: Point %d (%.2f, %.2f, %.2f) intersects with obstacle\n", 
                        i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
                }
            }
            free(traj->points);
            free(traj);
            free(occupation_map);
            //free(objective_points);
            free(size_map);

        }
        else{

            float* restrictions_map = map_main3D(occupation_map, size_map, distance_threshold, 
                                            objective_points, size_objective, start_points, size_start, 
                                            planner_type, escalado_vectores);
            // In your compute_3d_trajectory function:
            int initial_capacity = 100;
            Trajectory3D* traj = malloc(sizeof(Trajectory3D));
            if (traj == NULL) {
                printf("Error: Memory allocation failed for 3D trajectory.\n");
                return;
            }
            traj->points = malloc(initial_capacity * sizeof(Point3D));  // Initial capacity
            traj->size = 0;
            traj->capacity = initial_capacity;

            // Call th FMM2 function
            
            FMM2_3D(restrictions_map, size_map, distance_threshold, 
                objective_points, size_objective, start_points, size_start,
                step, traj, planner_type, escalado_vectores, occupation_map);

            clock_t end = clock();
            float cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
            printf("Tiempo de ejecución: %f segundos\n", cpu_time_used); 
            // Check if a trajectory was generated
            if (traj->size == 0) {
                printf("Error: No trajectory generated\n");
                free(traj->points);
                free(traj);
                free(occupation_map);
                free(start_points);
                free(size_map);
                return;
            }
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
            free(traj->points);
            free(traj);
            free(size_map);
        }
               
    }
    else if (dimensions_prob == 2){
        clock_t start = clock();
        
        // Define las dimensiones de la occupation_map
        const char* mapfile = "../Mapas/MAP_3_100_100.txt";
        int filas, columnas;
        int *size_map = (int *)malloc(2 * sizeof(int));

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int dimensions = 2;
        int num_start_points = 1;
        int size_start[2] = {dimensions,num_start_points};
        float *start_points = (float *)malloc(num_start_points * 2 * sizeof(float));;
        start_points[0] = 18;  // x coordinate
        start_points[1] = 50;  // y coordinate

        // Define las coordenadas objetivo
        int num_points = 1;
        int size_objective[2] = {dimensions,num_points};
        float *objective_points  = (float *)malloc(num_points * 2 * sizeof(float));;
        objective_points[0] = 82;  // x coordinate
        objective_points[1] = 20;  // y coordinate
        
        // PARSAMETROS DE LOS PLANNER
        int planner_type = 0;       // tipo de planner a usar
        int escalado_vectores = 4; // valor para escalar los vectores del planner 2

        // Define el umbral de distancia para la matriz de velocidades
        float distance_threshold = 4;

        // Define el tamaño del paso para el descenso del gradiente
        float step = 0.5;
        
        float *occupation_map = process_map_file((char*)mapfile, size_map, dimensions_prob);
        columnas = size_map[0];
        filas = size_map[1];
        // Check that initial and final points are not inside an obstacle
        if ((occupation_map[(int)start_points[0] - 1 + ((int)start_points[1]-1)*columnas] == 1)) {
            printf("Error: Initial point is inside an obstacle\n");
            return;
        }
        if ((occupation_map[(int)objective_points[0] - 1 + ((int)objective_points[1]-1)*columnas ] == 1)){
            printf("Error: Objective point is inside an obstacle\n");
            return;
        }
        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1]){ 
            printf("Error: Initial or objective point is outside the map\n");
            return;
        }
        float* restrictions_map = map_main2D(occupation_map, size_map, distance_threshold, 
                                            objective_points, size_objective, start_points, size_start, 
                                            planner_type, escalado_vectores);
        

        // Crear la trayectoria
        int initial_capacity = 100;
        Trajectory* traj = malloc(sizeof(Trajectory));
        traj->points = malloc(initial_capacity * sizeof(Point2D));
        traj->size = 0;
        traj->capacity = initial_capacity;
        FMM2_2D(restrictions_map, size_map, distance_threshold, 
                objective_points, size_objective, start_points, size_start, step, traj, planner_type, escalado_vectores);
        clock_t end = clock();
        float cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
        
        // Check if a trajectory was generated
        if (traj->size == 0) {
            printf("Error: No trajectory generated\n");
            free(traj->points);
            free(traj);
            free(size_map);
            return;
        }
        // Save trajectory to file
        /*
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
        */

        printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);
        free(traj->points);
        free(traj);
        free(size_map);
    }
    else{
        printf("Invalid number of dimensions\n");
    }
}
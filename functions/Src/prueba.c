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

#define SUCCESS 0
#define ERROR_INITIAL_OBSTACLE 1
#define ERROR_OBJECTIVE_OBSTACLE 2
#define ERROR_POINTS_OUTSIDE 3
#define ERROR_NO_TRAJECTORY 4
#define ERROR_COLLISION_DETECTED 5

// Add structure for results
typedef struct {
    // Input parameters
    float start_x, start_y, start_z; 
    float objective_x, objective_y, objective_z;
    float step;
    int planner_type;
    float distance_threshold;
    float escalado_vectores;
    float flight_level;
    // Results
    double execution_time;
    int error_code;
    int trajectory_points;
    char error_message[256];
} TestResult;

typedef struct {
    float start_x, start_y, start_z; 
    float objective_x, objective_y, objective_z;
    float step;
    float distance_threshold;
    float escalado_vectores;
    float flight_level;
    float planner_type;
} Parameters;

// Declare the prototype for fast_marching_2D
TestResult fast_marching(Parameters params, TestResult* result);

void main(){

    // Create results file with timestamp
    time_t now = time(NULL);
    struct tm *t = localtime(&now);
    char filename[256];
    sprintf(filename, "../Archivos/results_3D_100x100x100.csv");

    FILE* results_file = fopen(filename, "w");
    if (results_file == NULL) {
        printf("Error: Could not create results file\n");
        return;
    }

    // Write CSV header
    fprintf(results_file, "Combination,Start_X,Start_Y,Start_Z,Objective_X,Objective_Y,Objective_Z,Step,Planner_Type,"
            "Distance_Threshold,Escalado_Vectores,Flight_Level,"
            "Execution_Time,Trajectory_Points,Error_Code,Error_Message\n");


     FILE* param_file = fopen("../Archivos/parameter_combinations.txt", "r");
    if (param_file == NULL) {
        printf("Error: Could not open parameter combinations file\n");
        return;
    }

    // Read header
    int num_combinations, num_params;
    fscanf(param_file, "%d %d\n", &num_combinations, &num_params);

    // Skip parameter names line
    char line[1024];
    fgets(line, sizeof(line), param_file);

    // Process each combination
    for (int i = 0; i < num_combinations; i++) {
        Parameters params;
        
        int items_read = fscanf(param_file, 
        "%f %f %f %f %f %f %f %f %f %f %f", 
        &params.start_x, &params.start_y, &params.start_z,
        &params.objective_x, &params.objective_y, &params.objective_z,
        &params.step,
        &params.distance_threshold, &params.escalado_vectores,
        &params.flight_level,
        &params.planner_type);

        // Debug print
        printf("\nReading combination %d: %.2f %.2f %.2f %.2f %.2f %.2f %.2f %f %.2f %.2f %.2f\n",
            i+1, params.start_x, params.start_y, params.start_z, params.objective_x, params.objective_y, params.objective_z,
            params.step, params.distance_threshold, 
            params.escalado_vectores, params.flight_level, params.planner_type);

        // Convert float to int where needed after reading
        params.planner_type = (int)params.planner_type;
        params.flight_level = (int)params.flight_level;

        // Verify we read all parameters correctly
        if (items_read != 11) {
            printf("Error: Could not read all parameters for combination %d (read %d of 11)\n", 
                i+1, items_read);
            printf("Last successful values: start=(%.2f,%.2f, %.2f), obj=(%.2f,%.2f, %.2f), step=%.2f\n",
                params.start_x, params.start_y, params.start_z, params.objective_x, params.objective_y, params.objective_z, params.step);
            continue;
        }

        printf("\nRunning combination %d/%d:\n", i+1, num_combinations);
        printf("Start: (%.2f, %.2f, %.2f)\n", params.start_x, params.start_y, params.start_z);
        printf("Objective: (%.2f, %.2f, %.2f)\n", params.objective_x, params.objective_y, params.objective_z);
        printf("Parameters: step=%.2f, planner=%.2f, threshold=%.2f\n", 
               params.step, params.planner_type, params.distance_threshold);

        // Results
        TestResult result = {
            .start_x = params.start_x,
            .start_y = params.start_y,
            .start_z = params.start_z,
            .objective_x = params.objective_x,
            .objective_y = params.objective_y,
            .objective_z = params.objective_z,
            .step = params.step,
            .planner_type = params.planner_type,
            .distance_threshold = params.distance_threshold,
            .escalado_vectores = params.escalado_vectores,
            .flight_level = params.flight_level
        };

        fast_marching(params, &result);

        // Write results to CSV
        fprintf(results_file, "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%.2f,%.2f,%.2f,%.6f,%d,%d,\"%s\"\n",
            i+1,
            result.start_x, result.start_y, result.start_z,
            result.objective_x, result.objective_y, result.objective_z,
            result.step, result.planner_type,
            result.distance_threshold, result.escalado_vectores,
            result.flight_level,
            result.execution_time, result.trajectory_points,
            result.error_code, result.error_message);
    
        // Flush after each write to ensure data is saved
        fflush(results_file);
    }

    fclose(param_file);
    printf("\nAll combinations processed\n");

}


TestResult fast_marching(Parameters params, TestResult* result) {
    
    // Choose dimensions of the trayectory
    int dimensions_prob = 3;// Removed redefinition of 'dimensions'

    if (dimensions_prob == 3){
        clock_t start = clock();
        // Coord X = ancho, Y = largo, Z = alto

        //const char* mapfile = "./Mapas/MAP_3_100_100_100.txt";         
        const char* mapfile = "../Mapas/MAP_3_100_100_100.txt"; 
        //Procesar el mapa
        int* size_map = (int *)malloc(3 * sizeof(int));

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        int size_start[2] = {3, num_start_points};
        float *start_points = (float *)malloc(num_start_points * 3 * sizeof(float));;
        //start_points[0] = 18;  // x coordinate
        //start_points[1] = 50;  // y coordinate
        //start_points[2] = 12;   // z coordinate
        start_points[0] = round(params.start_x);
        start_points[1] = round(params.start_y);
        start_points[2] = round(params.start_z);   // z coordinate
        

        // Define las coordenadas objetivo
        int num_points = 1;
        // Removed redefinition of 'dimensions'
        int size_objective[2] = {3,1};
        float *objective_points  = (float *)malloc(num_points * 3 * sizeof(float));;
        //objective_points[0] = 82;  // x coordinate
        //objective_points[1] = 20;  // y coordinate
        objective_points[2] = 10;     // z coordinate
        objective_points[0] = round(params.objective_x);
        objective_points[1] = round(params.objective_y);
        objective_points[2] = round(params.objective_z);     // z coordinate

        // PARAMETROS PARA LOS PLANNER
        //int planner_type = 2;
        //int escalado_vectores = 5;      //valor para escalar los vectores del planner 1
        int ascension_rate = 1;         
        int descent_rate = 1;           
        //int flight_level = 70;          // Altura de vuelo en metros
        int resolution = 5;             // Resolution in meters per cell (1 cell = resolution meters)
        
        int planner_type = params.planner_type; 
        float escalado_vectores = params.escalado_vectores; 
        float flight_level = params.flight_level; 

        // Define el umbral de distancia para la matriz de velocidades
        //float distance_threshold = 8.0;
        float distance_threshold = params.distance_threshold;
        // Define el tamaño del paso
        //float step = 0.5;
        float step = params.step; 

        float *occupation_map = process_map_file((char*)mapfile, size_map, dimensions_prob);
        int ancho = size_map[0];
        int largo = size_map[1];
        int alto = size_map[2];

        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] || objective_points[2] > size_map[2] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1] || start_points[2] > size_map[2]){ 
            printf("Error: Initial or objective point is outside the map\n");
            result->error_code = ERROR_POINTS_OUTSIDE;
            strcpy(result->error_message, "Initial or objective point is outside the map");
            goto cleanup;
        }

        // Check that initial and final points are not inside an obstacle
        if ((occupation_map[(int)start_points[1] - 1 + ((int)start_points[0] -1)*largo + ((int)start_points[2] - 1)*ancho*largo] == 1)) {
            printf("Error: Start point is inside an obstacle\n");
            result->error_code = ERROR_INITIAL_OBSTACLE;
            strcpy(result->error_message, "Start point is inside an obstacle");
            goto cleanup;
        }
        if((occupation_map[(int)objective_points[1] - 1 + ((int)objective_points[0] - 1)*largo + ((int)objective_points[2] - 1)*ancho*largo] == 1)){
            printf("Objectve point is inside an obstacle\n");
            result->error_code = ERROR_OBJECTIVE_OBSTACLE;
            strcpy(result->error_message, "Objectve point is inside an obstacle");
            goto cleanup;
        }
        
        FILE* map_file = fopen("../Archivos/occupation_map.txt", "w");
        if (map_file == NULL) {
            printf("Error: Could not create occupation map file\n");
            //return;
        }
        fprintf(map_file, "%d %d %d\n", size_map[0], size_map[1], size_map[2]);
        // Write map data layer by layer
        for (int z = 0; z < size_map[2]; z++) {
            fprintf(map_file, "Layer %d:\n", z);
            for (int x = 0; x < size_map[1]; x++) {
                for (int y = 0; y < size_map[0]; y++) {
                    fprintf(map_file, "%.0f ", occupation_map[y + x*size_map[0] + z*size_map[0]*size_map[1]]);
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
                //return;
            }
            asc_restraint_planner(occupation_map, size_map, distance_threshold, 
                objective_points, size_objective, start_points, size_start,
                step, traj, planner_type, escalado_vectores, ascension_rate, 
                descent_rate, flight_level, resolution);

            clock_t end = clock();
            float cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
            printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);
            result->execution_time = cpu_time_used;
            result->trajectory_points = traj->size;
            // Check if a trajectory was generated
            if (traj->size == 0) {
                printf("Error: No trajectory generated\n");
                result->error_code = ERROR_NO_TRAJECTORY;
                //free(traj->points);
                //free(traj);
                //free(occupation_map);
                //free(size_map);
                return *result;
            }
            else{
                printf("\nChecking trajectory for obstacle collisions...\n");
                bool collision_found = false;
                for (int i = 0; i < traj->size; i++) {
                    int x = (int)round(traj->points[i].x) - 1;
                    int y = (int)round(traj->points[i].y) - 1;
                    int z = (int)round(traj->points[i].z) - 1;
                    
                    // Check if point is in obstacle (occupation_map has 1s for obstacles)
                    if (occupation_map[y + (x)*size_map[1] + (z)*size_map[0]*size_map[1]] == 1) {
                        printf("Warning: Point %d (%.2f, %.2f, %.2f) intersects with obstacle\n", 
                            i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
                        collision_found = true;
                    }
                }
                if (collision_found) {
                    result->error_code = ERROR_COLLISION_DETECTED;
                    strcpy(result->error_message, "Trajectory intersects with obstacles");
                } else {
                    result->error_code = SUCCESS;
                    strcpy(result->error_message, "Success - No collisions detected");
                }
            }
            free(traj->points);
            free(traj);
            free(occupation_map);
            //free(objective_points);
            free(size_map);
            return *result;

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
                //return;
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
            result->execution_time = cpu_time_used;
            result->trajectory_points = traj->size;
            // Check if a trajectory was generated
            if (traj->size == 0) {
                printf("Error: No trajectory generated\n");
                result->error_code = ERROR_NO_TRAJECTORY;
                free(traj->points);
                free(traj);
                free(occupation_map);
                free(start_points);
                free(size_map);
                return *result;
            }
            else{
                printf("\nChecking trajectory for obstacle collisions...\n");
                bool collision_found = false;
                for (int i = 0; i < traj->size; i++) {
                    int x = (int)round(traj->points[i].x) - 1;
                    int y = (int)round(traj->points[i].y) - 1;
                    int z = (int)round(traj->points[i].z) - 1;
                    
                    // Check if point is in obstacle (occupation_map has 1s for obstacles)
                    if (occupation_map[y + (x)*size_map[1] + (z)*size_map[0]*size_map[1]] == 1) {
                        printf("Warning: Point %d (%.2f, %.2f, %.2f) intersects with obstacle\n", 
                            i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
                        collision_found = true;
                    }
                }
                if (collision_found) {
                    result->error_code = ERROR_COLLISION_DETECTED;
                    strcpy(result->error_message, "Trajectory intersects with obstacles");
                } else {
                    result->error_code = SUCCESS;
                    strcpy(result->error_message, "Success - No collisions detected");
                }
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


        cleanup:
            free(size_map);
            free(occupation_map);
            free(objective_points);
            free(start_points);
            return *result;
        }
        //free(start_points);
        //free(objective_points);
               
    }
    else if (dimensions_prob == 2){
        clock_t start = clock();
        
        // Define las dimensiones de la occupation_map
        const char* mapfile = "../Mapas/MAP_2_50_50.txt";
        int filas, columnas;
        int *size_map = (int *)malloc(2 * sizeof(int));
        
        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int dimensions = 2;
        int num_start_points = 1;
        int size_start[2] = {dimensions,num_start_points};
        float *start_points = (float *)malloc(num_start_points * 2 * sizeof(float));;
        //start_points[0] = 18;  // x coordinate
        //start_points[1] = 50;  // y coordinate
        start_points[0] = round(params.start_x);
        start_points[1] = round(params.start_y);

        // Define las coordenadas objetivo
        int num_points = 1;
        int size_objective[2] = {dimensions,num_points};
        float *objective_points  = (float *)malloc(num_points * 2 * sizeof(float));;
        //objective_points[0] = 82;  // x coordinate
        //objective_points[1] = 20;  // y coordinate
        objective_points[0] = round(params.objective_x);
        objective_points[1] = round(params.objective_y);
        
        // PARAMETROS DE LOS PLANNER
        //int planner_type = 0;       // tipo de planner a usar
        //int escalado_vectores = 4; // valor para escalar los vectores del planner 2
        int planner_type = params.planner_type;
        int escalado_vectores = params.escalado_vectores;             
        // Define el umbral de distancia para la matriz de velocidades
        //float distance_threshold = 4;
        float distance_threshold = params.distance_threshold;

        // Define el tamaño del paso para el descenso del gradiente
        //float step = 0.5;
        float step = params.step;
        
        float *occupation_map = process_map_file((char*)mapfile, size_map, dimensions_prob);
        columnas = size_map[0];
        filas = size_map[1];
        // Check that initial and final points are not inside an obstacle
        if ((occupation_map[(int)start_points[0] - 1 + ((int)start_points[1]-1)*columnas] == 1)) {
            printf("Error: Initial point is inside an obstacle\n");
            result->error_code = ERROR_INITIAL_OBSTACLE;
            strcpy(result->error_message, "Initial point is inside an obstacle");
            goto cleanup2D;
        }
        if ((occupation_map[(int)objective_points[0] - 1 + ((int)objective_points[1]-1)*columnas ] == 1)){
            printf("Error: Objective point is inside an obstacle\n");
            result->error_code = ERROR_OBJECTIVE_OBSTACLE;
            strcpy(result->error_message, "Objectve point is inside an obstacle");
            goto cleanup2D;
        }
        if (objective_points[0] > size_map[0] || objective_points[1] > size_map[1] 
            ||start_points[0] > size_map[0] || start_points[1] > size_map[1]){ 
            printf("Error: Initial or objective point is outside the map\n");
            result->error_code = ERROR_POINTS_OUTSIDE;
            strcpy(result->error_message, "Initial or objective point is outside the map");
            goto cleanup2D;
        }
        float* restrictions_map = map_main2D(occupation_map, size_map, distance_threshold, 
                                            objective_points, size_objective, start_points, size_start, 
                                            planner_type, escalado_vectores, true);
        

        // Crear la trayectoria
        int initial_capacity = 100;
        Trajectory* traj = malloc(sizeof(Trajectory));
        traj->points = malloc(initial_capacity * sizeof(Point2D));
        traj->size = 0;
        traj->capacity = initial_capacity;
        FMM2_2D(restrictions_map, size_map, distance_threshold, 
                objective_points, size_objective, start_points, size_start, step, traj, planner_type, escalado_vectores);
        clock_t end = clock();
        double cpu_time_used = ((float) (end - start)) / CLOCKS_PER_SEC;
        result->execution_time = cpu_time_used;
        result->trajectory_points = traj->size;
        // Check if a trajectory was generated
        if (traj->size == 0) {
            printf("Error: No trajectory generated\n");
            result->error_code = ERROR_NO_TRAJECTORY;
            strcpy(result->error_message, "No trajectory generated");
            free(traj->points);
            free(traj);
            free(size_map);
            return *result;
        }
        result->error_code = SUCCESS;
        strcpy(result->error_message, "Success");
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

    cleanup2D:
        free(size_map);
        free(occupation_map);
        free(objective_points);
        free(start_points);
        return *result;
    }
    else{
        printf("Invalid number of dimensions\n");
    }
}
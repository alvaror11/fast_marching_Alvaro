#include <stdio.h>
#include <stdlib.h>
#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "common.h"
#include "rk4_2D_3D.h"
#include <time.h>
#include "FMM2.h"
#include <stdbool.h>



double* velocities_map3D(double* matriz, int* size_map, int distance_threshold);
double* main_msfm3D(double* obstacle_distance_map, double* objective_points, double* output_T, int* size_map, int* size_objective);
void compute_gradient_3d_discrete(double* input_matrix, double* gradient_matrix, int* size_map);
void compute_2d_trajectory();

void compute_2d_trajectory(){
    // Define las dimensiones de la matriz
    int filas = 50, columnas = 50; 
    

    // Define las coordenadas objetivo
    int num_points = 1;
    int dimensions = 2;
    int size_objective[2] = {2,1};
    double *objective_points  = (double *)malloc(num_points * 2 * sizeof(double));;
    objective_points[0] = 47;  // x coordinate
    objective_points[1] = 5;  // y coordinate

    //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
    int num_start_points = 1;
    double *start_points = (double *)malloc(num_start_points * 2 * sizeof(double));;
    start_points[0] = 5;  // x coordinate
    start_points[1] = 48;  // y coordinate

    // Define el umbral de distancia para la matriz de velocidades
    double distance_threshold = 4;
    double safety_margin = 2.5;

    // Define el tamaño del paso
    double step = 0.5;

    FILE *file = fopen("./Archivos/mapa.txt", "r");
    if (file == NULL) {
        perror("Error al abrir el archivo");
        return;
    }

    double *matriz = (double *)malloc(filas * columnas * sizeof(double));
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
    if ((matriz[(int)start_points[1] + (int)start_points[0]*columnas] == 1)||
        (matriz[(int)objective_points[1] + (int)objective_points[0]*columnas ] == 1)) {
        printf("Error: Initial or objective point is inside an obstacle\n");
        return;
    }
    //int size_map[2] = {columnas,filas};
    // Adding a layer of obstacles surrounding the map
    columnas = columnas+2;
    filas = filas+2;
    int size_map[2] = {columnas,filas};

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

    //Create velocities map
    
    double* obstacle_distance_map = velocities_map(matriz, size_map, distance_threshold, safety_margin);
        
    //Allocate memory for output map
    double* output_T = (double *)malloc(filas * columnas * sizeof(double));
    output_T = main_msfm(obstacle_distance_map, objective_points, output_T, size_map, size_objective);

    // Crear el gradiente para el mapa de tiempos:
    double* gradient_matrix = (double*)malloc(2 * filas * columnas * sizeof(double));
    compute_gradient_2d_discrete(output_T, gradient_matrix, size_map);

    // Empezamos a usar el descenso del gradiente para buscar el camino
    bool finished = false;      //mientras no se llegue al punto final es false
     
    // Crear la trayectoria
    int initial_capacity = 10;
    Trajectory* traj = malloc(sizeof(Trajectory));
    traj->points = malloc(initial_capacity * sizeof(Point2D));
    traj->size = 0;
    traj->capacity = initial_capacity;

    // Añadir el punto inicial
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
        }
    }

    
    // Liberar memoria    
    free(output_T);
    free(matriz);
    free(objective_points);
    free(start_points);
    free(obstacle_distance_map);
    free(last_point);
    free(traj);
    
}
void compute_3d_trajectory(){
    // Coord X = ancho, Y = largo, Z = alto
    int ancho = 50, largo = 50, alto = 50 ; 

    // Define las coordenadas objetivo
    int num_points = 1;
    // Removed redefinition of 'dimensions'
    int size_objective[2] = {3,1};
    double *objective_points  = (double *)malloc(num_points * 3 * sizeof(double));;
    objective_points[0] = 5;   // x coordinate
    objective_points[1] = 5;    // y coordinate
    objective_points[2] = 1;    // z coordinate

    //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
    int num_start_points = 1;
    double *start_points = (double *)malloc(num_start_points * 3 * sizeof(double));;
    start_points[0] = 47;    // x coordinate
    start_points[1] = 48;   // y coordinate
    start_points[2] = 35;   // z coordinate

    // Define el umbral de distancia para la matriz de velocidades
    double distance_threshold = 4.0;

    // Define el tamaño del paso
    double step = 0.5;

    FILE *file = fopen("./Archivos/mapa3D.txt", "r");
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
    if ((matriz[(int)start_points[0] + (int)start_points[1]*largo + (int)start_points[2]*ancho*largo] == 1)||
        (matriz[(int)objective_points[0] + (int)objective_points[1]*largo + (int)objective_points[2]*ancho*largo] == 1)) {
        printf("Error: Initial or objective point is inside an obstacle\n");
        return;
    }

    // Adding a layer of obstacles surrounding the map
    ancho = ancho+2;
    largo = largo+2;
    alto = alto+2;
    int size_map[3] = {ancho, largo, alto};

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

    //Create velocities map
    double* obstacle_distance_map = velocities_map3D(matriz, size_map, distance_threshold);

   
    //Allocate memory for output map
    double* output_T = (double *)malloc(ancho * largo * alto * sizeof(double));
    output_T = main_msfm3D(obstacle_distance_map, objective_points, output_T, size_map, size_objective);

 
    // Crear el gradiente para el mapa de tiempos:
    double* gradient_matrix = (double*)malloc(3 * ancho * largo * alto * sizeof(double));
    compute_gradient_3d_discrete(output_T, gradient_matrix, size_map);

    // Empezamos a usar el descenso del gradiente para buscar el camino
    bool finished = false;      //mientras no se llegue al punto final es false
    
    
    // Crear la trayectoria
    int initial_capacity = 100;
    Trajectory3D* traj = malloc(sizeof(Trajectory3D));
    traj->points = malloc(initial_capacity * sizeof(Point3D));
    traj->size = 0;
    traj->capacity = initial_capacity;

    // Añadir el punto inicial
    addPointToTrajectory3D(traj, start_points[0], start_points[1], start_points[2]);

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
    
    // Liberar memoria    
    free(output_T);
    free(matriz);
    free(objective_points);
    free(start_points);
    free(obstacle_distance_map);
    free(last_point);
    free(traj);
}

void main() {
    
    // Choose dimensions of the trayectory
     int dimensions_prob = 3; // Removed redefinition of 'dimensions'

    if (dimensions_prob == 3){
        // Coord X = ancho, Y = largo, Z = alto
        int ancho = 50, largo = 50, alto = 50 ; 

        // Define las coordenadas objetivo
        int num_points = 1;
        // Removed redefinition of 'dimensions'
        int size_objective[2] = {3,1};
        double *objective_points  = (double *)malloc(num_points * 3 * sizeof(double));;
        objective_points[0] = 5;   // x coordinate
        objective_points[1] = 5;    // y coordinate
        objective_points[2] = 1;    // z coordinate

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        double *start_points = (double *)malloc(num_start_points * 3 * sizeof(double));;
        start_points[0] = 47;    // x coordinate
        start_points[1] = 48;   // y coordinate
        start_points[2] = 35;   // z coordinate

        // Define el umbral de distancia para la matriz de velocidades
        double distance_threshold = 4.0;

        // Define el tamaño del paso
        double step = 0.5;

        FILE *file = fopen("./Archivos/mapa3D.txt", "r");
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
        if ((matriz[(int)start_points[0] + (int)start_points[1]*largo + (int)start_points[2]*ancho*largo] == 1)||
            (matriz[(int)objective_points[0] + (int)objective_points[1]*largo + (int)objective_points[2]*ancho*largo] == 1)) {
            printf("Error: Initial or objective point is inside an obstacle\n");
            return;
        }

        // Adding a layer of obstacles surrounding the map
        ancho = ancho+2;
        largo = largo+2;
        alto = alto+2;
        int size_map[3] = {ancho, largo, alto};

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

        // In your compute_3d_trajectory function:
        int initial_capacity = 100;
        Trajectory3D* traj = malloc(sizeof(Trajectory3D));
        traj->points = malloc(initial_capacity * sizeof(Point3D));  // Initial capacity
        traj->size = 0;
        traj->capacity = initial_capacity;

        // Call th FMM2 function
        clock_t start = clock();
        FMM2_3D(matriz, size_map, distance_threshold, ancho, largo, alto,
            objective_points, size_objective, start_points, step, traj);
        clock_t end = clock();
        double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
        printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);
        for(int i = 0; i < traj->size; i++) {
            printf("Point %d: (%.2f, %.2f, %.2f)\n", i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
        }
    }
    else if (dimensions_prob == 2){
        // Define las dimensiones de la matriz
        int filas = 50, columnas = 50; 
        
        // Define las coordenadas objetivo
        int num_points = 1;
        int dimensions = 2;
        int size_objective[2] = {2,1};
        double *objective_points  = (double *)malloc(num_points * 2 * sizeof(double));;
        objective_points[0] = 47;  // x coordinate
        objective_points[1] = 5;  // y coordinate

        //Define las coordenadas de inicio, por ahora solo funciona con un punto inicial
        int num_start_points = 1;
        double *start_points = (double *)malloc(num_start_points * 2 * sizeof(double));;
        start_points[0] = 5;  // x coordinate
        start_points[1] = 48;  // y coordinate

        // Define el umbral de distancia para la matriz de velocidades
        double distance_threshold = 4;
        double safety_margin = 2.5;

        // Define el tamaño del paso
        double step = 0.5;

        FILE *file = fopen("./Archivos/mapa.txt", "r");
        if (file == NULL) {
            perror("Error al abrir el archivo");
            return;
        }

        double *matriz = (double *)malloc(filas * columnas * sizeof(double));
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
        if ((matriz[(int)start_points[1] + (int)start_points[0]*columnas] == 1)||
            (matriz[(int)objective_points[1] + (int)objective_points[0]*columnas ] == 1)) {
            printf("Error: Initial or objective point is inside an obstacle\n");
            return;
        }
        // Adding a layer of obstacles surrounding the map
        columnas = columnas+2;
        filas = filas+2;
        int size_map[2] = {columnas,filas};

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
        clock_t start = clock();
        // Crear la trayectoria
        int initial_capacity = 10;
        Trajectory* traj = malloc(sizeof(Trajectory));
        traj->points = malloc(initial_capacity * sizeof(Point2D));
        traj->size = 0;
        traj->capacity = initial_capacity;
        FMM2_2D(matriz, size_map, distance_threshold, safety_margin, filas, columnas, 
                objective_points, size_objective, start_points, step, traj);
        clock_t end = clock();
        // Use the trajectory
        for(int i = 0; i < traj->size; i++) {
            printf("Point %d: (%.2f, %.2f)\n", i, traj->points[i].x, traj->points[i].y);
        }
        double cpu_time_used = ((double) (end - start)) / CLOCKS_PER_SEC;
        printf("Tiempo de ejecución: %f segundos\n", cpu_time_used);
    }
    else{
        printf("Invalid number of dimensions\n");
    }
}
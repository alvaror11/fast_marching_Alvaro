#include <stdio.h>
#include <stdlib.h>
#include "msfm2d_MOD.h"
#include "msfm3d_MOD.h"
#include "common.h"
#include "rk4_2D_3D.h"

double* velocities_map3D(double* matriz, int* size_map, int distance_threshold);
double* main_msfm3D(double* obstacle_distance_map, double* objective_points, double* output_T, int* size_map, int* size_objective);
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
    double distance_threshold = 8;
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

    FILE *surrounded_map_file = fopen("./Archivos/surrounded_map.txt", "w");
    if (surrounded_map_file == NULL) {
        perror("Error al abrir el archivo del mapa rodeado");
        return;
    }

    // Write the surrounded map to file
    printf("\nSaving surrounded map with dimensions: %d x %d\n", filas, columnas);
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            fprintf(surrounded_map_file, "%.0f ", matriz2[j + i * columnas]);
        }
        fprintf(surrounded_map_file, "\n");
    }

    fclose(surrounded_map_file);
    free(matriz);
    matriz = matriz2;
    

    //Create velocities map
    double* obstacle_distance_map = velocities_map(matriz, size_map, distance_threshold, safety_margin);
    
    //GUARDAR LA MATRIZ DE VELOCIDADES EN UN ARCHIVO DE SALIDA
    FILE *output_file1 = fopen("./Archivos/velocities_map.txt", "w");
    if (output_file1 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }

    // Escribir los resultados del mapa de velocidades en el archivo de salida
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            fprintf(output_file1, "%.2f ", obstacle_distance_map[j + i * columnas]);
        }
        fprintf(output_file1, "\n");
    }
    fclose(output_file1);
    
    //Allocate memory for output map
    double* output_T = (double *)malloc(filas * columnas * sizeof(double));
    output_T = main_msfm(obstacle_distance_map, objective_points, output_T, size_map, size_objective);


    //GUARDAR LA MATRIZ DE TIEMPOS EN UN ARCHIVO DE SALIDA
    FILE *output_file2 = fopen("./Archivos/times_map.txt", "w");
    if (output_file2 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }

    // Escribir los resultados del mapa de distancias en el archivo de salida
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            fprintf(output_file2, "%.2f ", output_T[j + i * columnas]);
        }
        fprintf(output_file2, "\n");
    }
    fclose(output_file2);

    // Crear el gradiente para el mapa de tiempos:
    double* gradient_matrix = (double*)malloc(2 * filas * columnas * sizeof(double));
    compute_gradient_2d_discrete(output_T, gradient_matrix, size_map);

    // After compute_gradient_2d_discrete call
    // Export gradient components to separate files
    FILE *gradient_x_file = fopen("./Archivos/gradient_x.txt", "w");
    FILE *gradient_y_file = fopen("./Archivos/gradient_y.txt", "w");

    if (gradient_x_file == NULL || gradient_y_file == NULL) {
        perror("Error al abrir los archivos de gradiente");
        return;
    }

    // Write gradient components in matrix format
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            // Write x component
            fprintf(gradient_x_file, "%.4f ", gradient_matrix[i*columnas + j]);
            // Write y component
            fprintf(gradient_y_file, "%.4f ", gradient_matrix[i*columnas + j + filas*columnas]);
        }
        fprintf(gradient_x_file, "\n");
        fprintf(gradient_y_file, "\n");
    }

    fclose(gradient_x_file);
    fclose(gradient_y_file);




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
        
        // Print current trajectory
        printf("\nCurrent trajectory (size: %d):\n", traj->size);
        for (int i = 0; i < traj->size; i++) {
            printf("Point %d: (%.2f, %.2f)\n", i, traj->points[i].x, traj->points[i].y);
        }
        printf("-------------------------\n");

        // Comprobar si se ha llegado al punto objetivo
        if ((round(new_point[0]) == objective_points[0] || floor(new_point[0]) == objective_points[0]) 
            && (round(new_point[1]) == objective_points[1] || floor(new_point[1]) == objective_points[1])) {
            finished = true;
            traj->points[traj->size - 1].x = objective_points[0];
            traj->points[traj->size - 1].y = objective_points[1];
        }
    }

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
    free(obstacle_distance_map);
    free(last_point);
    free(traj);
}
void compute_3d_trajectory(){
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

    printf("\nReading 3D map with dimensions: %d x %d x %d\n", ancho, largo, alto);

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
    printf("\nCreating surrounded 3D map with dimensions: %d x %d x %d\n", ancho, largo, alto);

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

    // Save surrounded map for verification
    FILE *surrounded_map_file = fopen("./Archivos/surrounded_map3D.txt", "w");
    if (surrounded_map_file == NULL) {
        perror("Error al abrir el archivo del mapa rodeado 3D");
        return;
    }

    // Write the surrounded map to file
    printf("Saving surrounded 3D map...\n");
    for (int k = 0; k < alto; k++) {
        for (int i = 0; i < ancho; i++) {
            for (int j = 0; j < largo; j++) {
                fprintf(surrounded_map_file, "%.0f ", matriz2[j + i*largo + k*ancho*largo]);
            }
            fprintf(surrounded_map_file, "\n");
        }
        fprintf(surrounded_map_file, "\n");
    }

    fclose(surrounded_map_file);
    free(matriz);
    matriz = matriz2;

    //Create velocities map
    
    double* obstacle_distance_map = velocities_map3D(matriz, size_map, distance_threshold);
    
    //GUARDAR LA MATRIZ DE VELOCIDADES EN UN ARCHIVO DE SALIDA
    FILE *output_file1 = fopen("./Archivos/velocities_map3D.txt", "w");
    if (output_file1 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }

    // Escribir los resultados del mapa de velocidades en el archivo de salida
    for (int k = 0; k < alto; k++) {
        for (int i = 0; i < ancho; i++) {
            for (int j = 0; j < largo; j++) {
                fprintf(output_file1, "%.2f ", obstacle_distance_map[j + i * largo + k*ancho*largo]);
            }
            fprintf(output_file1, "\n");
        }
        
    }
    fclose(output_file1);
   
    //Allocate memory for output map
    double* output_T = (double *)malloc(ancho * largo * alto * sizeof(double));
    output_T = main_msfm3D(obstacle_distance_map, objective_points, output_T, size_map, size_objective);


    //GUARDAR LA MATRIZ DE TIEMPOS EN UN ARCHIVO DE SALIDA
    FILE *output_file2 = fopen("./Archivos/times_map3D.txt", "w");
    if (output_file2 == NULL) {
        perror("Error al abrir el archivo de salida");
        return;
    }

    // Escribir los resultados del mapa de distancias en el archivo de salida
    for (int k = 0; k < alto; k++) {
        for (int i = 0; i < ancho; i++) {
            for (int j = 0; j < largo; j++) {
                int index = j + i * largo + k * ancho * largo;
                if (index >= 0 && index < ancho * largo * alto) {
                    fprintf(output_file2, "%.2f ", output_T[index]);
                } else {
                    fprintf(output_file2, "0.00 ");  // Safe default value
                    printf("Warning: Invalid index access at k=%d, i=%d, j=%d\n", k, i, j);
                }
            }
        }
        fprintf(output_file2, "\n");
    }
    fclose(output_file2);
 
    // Crear el gradiente para el mapa de tiempos:
    double* gradient_matrix = (double*)malloc(3 * ancho * largo * alto * sizeof(double));
    compute_gradient_3d_discrete(output_T, gradient_matrix, size_map);

    // After compute_gradient_2d_discrete call
    // Export gradient components to separate files
    FILE *gradient_x_file = fopen("./Archivos/gradient3D_x.txt", "w");
    FILE *gradient_y_file = fopen("./Archivos/gradient3D_y.txt", "w");
    FILE *gradient_z_file = fopen("./Archivos/gradient3D_z.txt", "w");

    if (gradient_x_file == NULL || gradient_y_file == NULL || gradient_z_file == NULL) {
        perror("Error al abrir los archivos de gradiente");
        return;
    }

    // Write gradient components in matrix format
    for (int k = 0; k < alto; k++) {
        for (int i = 0; i < ancho; i++) {
            for (int j = 0; j < largo; j++) {
                // Write x component
                fprintf(gradient_x_file, "%.4f ", gradient_matrix[j + i * largo + k*ancho*largo]);
                // Write y component
                fprintf(gradient_y_file, "%.4f ", gradient_matrix[j + i * largo + k*ancho*largo + ancho*largo*alto]);
                // Write z component
                fprintf(gradient_z_file, "%.4f ", gradient_matrix[j + i * largo + k*ancho*largo + 2*ancho*largo*alto]);
            }
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
        
        printf("Last point coordinates: (%.2f, %.2f, %.2f)\n", last_point[0], last_point[1], last_point[2]);
        int index = (int)(last_point[1] + (int)last_point[0] * largo + (int)last_point[2] * ancho * largo);
        printf("Gradient values at point:\n");
        printf("X: %.4f\n", gradient_matrix[index]);
        printf("Y: %.4f\n", gradient_matrix[index + ancho*largo*alto]);
        printf("Z: %.4f\n", gradient_matrix[index + 2*ancho*largo*alto]);
        printf("-------------------------\n");
        // Llamar a la función mexFunction en rk4_2D con las coordenadas del último punto y el mapa de velocidades
        gradient_descend_rk4(last_point, gradient_matrix, size_map, size_objective, new_point, step);

        // Añadir el nuevo punto a la trayectoria
        addPointToTrajectory3D(traj, new_point[0], new_point[1], new_point[2]);
        
        // Print current trajectory
        printf("\nCurrent trajectory (size: %d):\n", traj->size);
        for (int i = 0; i < traj->size; i++) {
            printf("Point %d: (%.2f, %.2f, %.2f)\n", i, traj->points[i].x, traj->points[i].y, traj->points[i].z);
        }
        printf("-------------------------\n");

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

    FILE *traj_file = fopen("./Archivos/trajectory3D.txt", "w");
    if (traj_file == NULL) {
        perror("Error al abrir el archivo de trayectoria");
        return;
    }

    for (int i = 0; i < traj->size; i++) {
        double traj_x = traj->points[i].x;
        double traj_y = traj->points[i].y;
        double traj_z = traj->points[i].z;
        // Save to file (x y format)
        fprintf(traj_file, "%.2f %.2f %.2f\n", traj_x, traj_y, traj_z);
    }

    fclose(traj_file);

    // Liberar memoria    
    free(output_T);
    free(matriz);
    free(objective_points);
    free(start_points);
    free(obstacle_distance_map);
    free(last_point);
    free(traj);
}

int main() {
    
    // Choose dimensions of the trayectory
     int dimensions_prob = 3; // Removed redefinition of 'dimensions'

    if (dimensions_prob == 3){
        compute_3d_trajectory();
    }
    else if (dimensions_prob == 2){
        compute_2d_trajectory();
    }
    else{
        printf("Invalid number of dimensions\n");
    }
}
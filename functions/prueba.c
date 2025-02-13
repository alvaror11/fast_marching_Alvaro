#include <stdio.h>
#include <stdlib.h>
#include "msfm2d_MOD.h"
#include "common.h"

// Estructura de los puntos de la trayectoria
typedef struct {
    double x;
    double y;
} Point2D;

//Estructura de la trayectoria
typedef struct {
    Point2D* points;
    int size;
    int capacity;
} Trajectory;

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

int main() {

    // Define las dimensiones de la matriz
    int filas = 50, columnas = 50; 

    // Define las coordenadas objetivo
    int num_points = 1;
    double *objective_points  = (double *)malloc(num_points * 2 * sizeof(double));;
    objective_points[0] = 25;  // x coordinate
    objective_points[1] = 25;  // y coordinate

    //Define las coordenadas de inicio
    int num_start_points = 1;
    double *start_points = (double *)malloc(num_start_points * 2 * sizeof(double));;
    start_points[0] = 0;  // x coordinate
    start_points[1] = 0;  // y coordinate

    // Define el tamaño del paso
    int step = 1;

    FILE *file = fopen("mapa.txt", "r");
    if (file == NULL) {
        perror("Error al abrir el archivo");
        return 1;
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

    printf("\nMatriz leída del archivo:\n");
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            if (matriz[j + i * columnas] == 1) {
                matriz[j + i * columnas] = 0;
            } else if (matriz[j + i * columnas] == 0) {
                matriz[j + i * columnas] = 1;
            }
            printf("%.1f ",  matriz[j + i * columnas]);
        }
        printf("\n");
    }
     

    //Allocate memory for output map
    double* output_T = (double *)malloc(filas * columnas * sizeof(double));
    output_T = main_msfm(matriz, objective_points, output_T);

    // Print the distance map results
    printf("\nDistance map results:\n");
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            printf("%.2f ", output_T[i + j * filas]);
        }
        printf("\n");
    }
    /* GUARDAR LA MATRIZ DE DISTANCIAS EN UN ARCHIVO DE SALIDA
    FILE *output_file = fopen("output.txt", "w");
    if (output_file == NULL) {
        perror("Error al abrir el archivo de salida");
        return 1;
    }

    // Escribir los resultados del mapa de distancias en el archivo de salida
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            fprintf(output_file, "%.2f ", output_T[j + i * columnas]);
        }
        fprintf(output_file, "\n");
    }

    // Cerrar el archivo de salida
    fclose(output_file);*/

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
    

    while (finished = false);{
        // Obtener las coordenadas del último punto de la trayectoria
        double last_x = traj->points[traj->size - 1].x;
        double last_y = traj->points[traj->size - 1].y;

        //Definir el pointer para los nuevos puntos de la trayectoria
        double* new_point = malloc(2 * sizeof(double));

        // Llamar a la función mexFunction en rk4_2D con las coordenadas del último punto y el mapa de velocidades
        gradient_descend_rk4(last_x, last_y, matriz, filas, columnas, new_point, step);

        // Añadir el nuevo punto a la trayectoria
        addPointToTrajectory(traj, new_point[0], new_point[1]);

        // Comprobar si se ha llegado al punto objetivo
        if (new_point[0] == objective_points[0] && new_point[1] == objective_points[1]) {
            finished = true;
        }


    }


}
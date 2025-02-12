#include <stdio.h>
#include <stdlib.h>
#include "msfm2d_MOD.h"
#include "common.h"


int main() {
    FILE *file = fopen("mapa.txt", "r");
    if (file == NULL) {
        perror("Error al abrir el archivo");
        return 1;
    }

    int filas = 50, columnas = 50; // Define las dimensiones de la matriz (si no se conocen, deben calcularse)
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
    //printf("\n%.1f ",  matriz[38 + 47 * columnas]);
    // Define number of source points
    int num_points = 1;  // Example with one point
    
    // Allocate memory for source points array
    double *source_points  = (double *)malloc(num_points * 2 * sizeof(double));;
    
    // Set source point coordinates (example with one point at position [25,25])
    source_points[0] = 25;  // x coordinate
    source_points[1] = 25;  // y coordinate

    /* Imprimir la matriz para verificar */
    

    //Allocate memory for output map
    double* output_T = (double *)malloc(filas * columnas * sizeof(double));
    output_T = main_msfm(matriz, source_points, output_T);

    // Print the distance map results
    printf("\nDistance map results:\n");
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            printf("%.2f ", output_T[i + j * filas]);
        }
        printf("\n");
    }
    FILE *output_file = fopen("output.txt", "w");
    if (output_file == NULL) {
        perror("Error al abrir el archivo de salida");
        return 1;
    }

    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            fprintf(output_file, "%.2f ", output_T[j + i * columnas]);
        }
        fprintf(output_file, "\n");
    }

    fclose(output_file);


    /* Imprimir la matriz para verificar
    printf("Matriz leída del archivo:\n");
    for (int i = 0; i < filas; i++) {
        for (int j = 0; j < columnas; j++) {
            printf("%d ",  matriz[i + j * filas]);
        }
        printf("\n");
    } */

}
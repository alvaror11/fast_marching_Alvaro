#ifndef RK4_2D_H
#define RK4_2D_H
#include <stdbool.h>

void gradient_descend_rk4(double* point, double *matriz, int filas, int columnas, double *new_point, double step);

#endif 
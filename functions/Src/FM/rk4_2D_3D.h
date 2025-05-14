#ifndef RK4_2D_H
#define RK4_2D_H
#include <stdbool.h>

void gradient_descend_rk4(float* point, float *matriz, int* size_map, int* size_point, float *new_point, float step);

#endif 
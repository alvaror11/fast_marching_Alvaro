#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "common.h"


void planners_2D(double* matriz, int size_map[2], double* objective_points, int size_objective[2], 
                double* start_points, int planner_type){

    // Choose type of planner
    switch(planner_type){
        case 1:
            // Coger el cuadrado mas pequeño que contega puntos objetivo y de inicio
            int x_max = 0;
            int y_max = 0;
            int x_min = size_map[0];
            int y_min = size_map[1];
            // Find max and min coordinates for objective points
            for (int i = 0; i < size_objective[1]; i++){
                int x = (int) objective_points[i*2];
                int y = (int) objective_points[i*2 + 1];
                if (x > x_max) x_max = x;
                if (y > y_max) y_max = y;
                if (x < x_min) x_min = x;
                if (y < y_min) y_min = y;
            }

            // Find max and min coordinates for start points
            for (int i = 0; i < size_objective[1]; i++){
                int x = (int) start_points[i*2];
                int y = (int) start_points[i*2 + 1];
                if (x > x_max) x_max = x;
                if (y > y_max) y_max = y;
                if (x < x_min) x_min = x;
                if (y < y_min) y_min = y;
            }
            
            double dx = x_max - x_min;
            double dy = y_max - y_min;
            





}
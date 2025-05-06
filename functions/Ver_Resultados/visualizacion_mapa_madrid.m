clear;
close all;
clc;

main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";

cd(maps_folder)

mapaMAD = readmatrix("MADRIDALTMAP.csv");
surf(mapaMAD,"EdgeColor","none")  
axis equal

cd(main_folder)
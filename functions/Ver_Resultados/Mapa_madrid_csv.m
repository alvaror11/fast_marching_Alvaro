clear;
clc;
close all;

main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";

cd(maps_folder)

mapa = readmatrix("MADRIDALTMAP.CSV");

figure;
surf(mapa,"EdgeColor","none")
axis equal;
[max_alt, max_idx] = max(mapa(:));

cd(main_folder)
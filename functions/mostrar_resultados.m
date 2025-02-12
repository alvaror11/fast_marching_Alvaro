clear;
clc;

filas = 50;
columnas = 50;

file = fopen('output.txt','r');

matriz = zeros(filas, columnas); 

% Leer los datos línea por línea y almacenarlos en la matriz
for i = 1:filas
    linea = fgetl(file); % Leer una línea del archivo como string
    if ischar(linea)
        line = sscanf(linea, '%f')';
        matriz(i, :) = sscanf(linea, '%f')'; % Convertir la línea a números y almacenarla
    else
        error('Se alcanzó el final del archivo antes de leer todas las filas.');
    end
end

% Cerrar el archivo
fclose(file);


umbralMax = 40; 

% Crear una copia de la matriz con valores limitados a 200
matrizClipped = matriz;  
matrizClipped(matriz > umbralMax) = umbralMax; 
% Graficar la matriz con imágenes de escala de colores
figure;
imagesc(matrizClipped);  
colorbar; % Agregar barra de colores para referencia

% Ajustar la escala de colores con valores limitados
clim([min(matriz(:)), umbralMax]); 

% Usar un colormap donde los valores pequeños sean cálidos y los grandes fríos
colormap(jet);  % Otras opciones: parula, turbo, hot

% Etiquetas y título
title('Mapa de Colores de la Matriz');
xlabel('Columnas');
ylabel('Filas');

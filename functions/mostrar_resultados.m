clear;
clc;

filas = 50;
columnas = 50;

%% Mapa de velocidades
file = fopen('velocities_map.txt','r');

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


% Crear figura
figure;

% Usar colormap en escala de grises
colormap(gray);  % Usar escala de grises: negro (0) a blanco (1)

% Mostrar el mapa
imagesc(matriz);
axis equal;  % Mantener proporciones cuadradas
colorbar;
clim([0 1]);  % Ajustar límites de color entre 0 y 1

% Etiquetas y título
title('Mapa de Velocidades', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);


%% Mapa de tiempos
file = fopen('times_map.txt','r');

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


umbralMax = 200; 

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

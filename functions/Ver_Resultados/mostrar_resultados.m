clear;
clc;
close all;

filas = 50;
columnas = 50;

% Punto para iniciar la traj.
punto_x = 20;  % columna
punto_y = 22; % fila
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";

%% Mapa Original
cd(files_folder);
file = fopen('mapa.txt','r');

matriz_vel = zeros(filas, columnas); 

% Leer los datos línea por línea y almacenarlos en la matriz
for i = 1:filas
    linea = fgetl(file); % Leer una línea del archivo como string
    if ischar(linea)
        line = sscanf(linea, '%f')';
        matriz_vel(i, :) = sscanf(linea, '%f')'; % Convertir la línea a números y almacenarla
    else
        error('Se alcanzó el final del archivo antes de leer todas las filas.');
    end
end

% Cerrar el archivo
fclose(file);
cd(main_folder);

% Crear figura
figure;

% Usar colormap en escala de grises
colormap(gray);  % Usar escala de grises: negro (0) a blanco (1)

% Mostrar el mapa
imagesc(matriz_vel);
axis equal;  % Mantener proporciones cuadradas
%colorbar;
%clim([0 1]);  % Ajustar límites de color entre 0 y 1

% Etiquetas y título
title('Occupation Map', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);

%% Mapa de velocidades
cd(files_folder);
file = fopen('velocities_map.txt','r');

%solo sumar filas y columnas si no se usan planners
%filas = filas + 2;
%columnas = columnas + 2;
matriz_vel = zeros(filas, columnas); 

% Leer los datos línea por línea y almacenarlos en la matriz
for i = 1:filas
    linea = fgetl(file); % Leer una línea del archivo como string
    if ischar(linea)
        line = sscanf(linea, '%f')';
        matriz_vel(i, :) = sscanf(linea, '%f')'; % Convertir la línea a números y almacenarla
    else
        error('Se alcanzó el final del archivo antes de leer todas las filas.');
    end
end

% Cerrar el archivo
fclose(file);
cd(main_folder);

% Crear figura
figure;

% Usar colormap en escala de grises
colormap(gray);  % Usar escala de grises: negro (0) a blanco (1)

% Mostrar el mapa
imagesc(matriz_vel);
axis equal;  % Mantener proporciones cuadradas
colorbar;
clim([0 1]);  % Ajustar límites de color entre 0 y 1

% Etiquetas y título
title('Velocities Map', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);


%% Mapa de tiempos
cd(files_folder);
file = fopen('times_map.txt','r');

matriz_tiempos = zeros(filas, columnas); 

% Leer los datos línea por línea y almacenarlos en la matriz
for i = 1:filas
    linea = fgetl(file); % Leer una línea del archivo como string
    if ischar(linea)
        line = sscanf(linea, '%f')';
        matriz_tiempos(i, :) = sscanf(linea, '%f')'; % Convertir la línea a números y almacenarla
    else
        error('Se alcanzó el final del archivo antes de leer todas las filas.');
    end
end

% Cerrar el archivo
fclose(file);

cd(main_folder);

%% Mapa de tiempos visualization

umbralMax = 200; 

% Crear una copia de la matriz con valores limitados a 200
matrizClipped = matriz_tiempos;  
matrizClipped(matriz_tiempos > umbralMax) = umbralMax; 
% Graficar la matriz con imágenes de escala de colores
% ...existing code...


% Create figure for time map
figure;

% Mostrar el mapa
imagesc(matrizClipped);
colorbar;
colormap(jet);
clim([0 200]);

% Etiquetas y título
title('Times Map', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);
axis equal;

%% Trayectoria 
cd(files_folder);
traj_data = load('trajectory.txt');
cd(main_folder);

% Extract coordinates
traj_x = traj_data(:,1) + 1;
traj_y = traj_data(:,2) + 1;

% Convert to double if necessary
traj_x = double(traj_x);
traj_y = double(traj_y);

hold on;

% Plot path line
plot(traj_x, traj_y, 'w-', 'LineWidth', 2);

% Plot waypoints
plot(traj_x(2:end-1), traj_y(2:end-1), 'w.', 'MarkerSize', 10);

% Plot start point (green) and end point (red)
plot(traj_x(1), traj_y(1), 'go', 'MarkerSize', 15, 'LineWidth', 2);
plot(traj_x(end), traj_y(end), 'ro', 'MarkerSize', 15, 'LineWidth', 2);

% Add legend
legend('Path', 'Waypoints', 'Start', 'Goal', 'Location', 'southeast');

hold off;



%[Fy,Fx,Fz]=pointmin(matriz_tiempos);
% After loading time map and before gradient visualization

%% Load gradient components
cd(files_folder);
gradient_x_file = fopen('gradient_x.txt', 'r');
gradient_y_file = fopen('gradient_y.txt', 'r');

Fx = zeros(filas, columnas);
Fy = zeros(filas, columnas);

% Read gradient x component
for i = 1:filas
    linea = fgetl(gradient_x_file);
    if ischar(linea)
        Fx(i, :) = sscanf(linea, '%f')';
    else
        error('Error reading gradient_x file');
    end
end

% Read gradient y component
for i = 1:filas
    linea = fgetl(gradient_y_file);
    if ischar(linea)
        Fy(i, :) = sscanf(linea, '%f')';
    else
        error('Error reading gradient_y file');
    end
end

fclose(gradient_x_file);
fclose(gradient_y_file);
cd(main_folder);
% After existing time map visualization
% Create new figure for gradient field visualization
figure;
imagesc(matrizClipped);
hold on;

% Create grid for quiver plot
[X, Y] = meshgrid(1:columnas, 1:filas);

% Downsample for better visualization (adjust skip value as needed)
skip = 1;  % Show an arrow every 2 points
X = X(1:skip:end, 1:skip:end);
Y = Y(1:skip:end, 1:skip:end);
Fx_down = Fx(1:skip:end, 1:skip:end);
Fy_down = Fy(1:skip:end, 1:skip:end);

% Plot vector field
quiver(X, Y, Fx_down, Fy_down, 2, 'w', 'LineWidth', 1.5);

% Adjust colormap and limits
colormap(jet);
clim([min(matriz_tiempos(:)), umbralMax]);
colorbar;


% Add to legend
legend('Gradient field', 'Location', 'southeast');

% Add labels and title
title('Mapa de tiempos con campo de gradiente');
xlabel('X');
ylabel('Y');
axis equal;

hold off;

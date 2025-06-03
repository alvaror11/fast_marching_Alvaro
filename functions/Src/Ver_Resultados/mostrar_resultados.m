clear;
clc;
close all;

files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Src\Ver_Resultados";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";

%% Mapa Original
cd(maps_folder);
filename = 'MAP_3_100_100.txt';
file = fopen(filename,'r');

tokens = regexp(filename, 'MAP_(\d+)_(\d+)_(\d+)', 'tokens');
if ~isempty(tokens)
    % Extract dimensions from filename
    dims = str2double(tokens{1});
    filas = dims(2);     % number of rows
    columnas = dims(3);  % number of columns
else
    error('Could not extract dimensions from filename');
end

matriz_occ = zeros(filas, columnas); 

% Leer los datos línea por línea y almacenarlos en la matriz
for i = 1:filas
    linea = fgetl(file); % Leer una línea del archivo como string
    if ischar(linea)
        line = sscanf(linea, '%f')';
        matriz_occ(i, :) = sscanf(linea, '%f')'; % Convertir la línea a números y almacenarla
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
imagesc(matriz_occ);
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
fileID = fopen('velocities_map.txt', 'r');
if fileID == -1
    error('Could not open velocities map file');
end

% Read dimensions from first line
dims = fscanf(fileID, '%d', [1 2]);  % Read two dimensions [width height]
velocities_map = zeros(dims(2), dims(1));
filas = dims(2);
columnas = dims(1);
fgetl(fileID);
% Read the map data
for i = 1:dims(2)
    line = fgetl(fileID);
    if ~ischar(line)
        error(['Error reading line ' num2str(i)]);
    end
    values = str2num(line);
    long = length(values);
    if length(values) ~= dims(1)
        error(['Incorrect number of values in line ' num2str(i)]);
    end
    velocities_map(i,:) = values;
end

fclose(fileID);
cd(main_folder);

% Crear figura
figure;

% Usar colormap en escala de grises
colormap(gray);  % Usar escala de grises: negro (0) a blanco (1)

% Mostrar el mapa
imagesc(velocities_map);
axis equal;  % Mantener proporciones cuadradas
%colorbar;
%clim([0 1]);  % Ajustar límites de color entre 0 y 1

% Etiquetas y título
title('Velocities Map', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);

%% Mapa de restricciones
cd(files_folder);
fileID1 = fopen('restrictions_map.txt', 'r');
if fileID1 == -1
    error('Could not open restrictions map file');
end

% Read dimensions from first line
dims = fscanf(fileID1, '%d', [1 2]);  % Read two dimensions [width height]
restrictions_map = zeros(dims(2), dims(1));
filas = dims(2);
columnas = dims(1);
fgetl(fileID1);
% Read the map data
for i = 1:dims(2)
    line = fgetl(fileID1);
    if ~ischar(line)
        error(['Error reading line ' num2str(i)]);
    end
    values = str2num(line);
    long = length(values);
    if length(values) ~= dims(1)
        error(['Incorrect number of values in line ' num2str(i)]);
    end
    restrictions_map(i,:) = values;
end

fclose(fileID1);
cd(main_folder);

% Crear figura
figure;

% Usar colormap en escala de grises
colormap(gray);  % Usar escala de grises: negro (0) a blanco (1)

% Mostrar el mapa
imagesc(restrictions_map);
axis equal;  % Mantener proporciones cuadradas
%colorbar;
%clim([0 1]);  % Ajustar límites de color entre 0 y 1

% Etiquetas y título
title('Restrictions Map', 'FontSize', 12);
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

umbralMax = 400; 

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
clim([0 umbralMax]);

% Etiquetas y título
title('Trajectory and times map', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);
axis equal;

%% Trayectoria 
cd(files_folder);
traj_data = load('trajectory.txt');
cd(main_folder);

% Extract coordinates
traj_x = traj_data(:,1);
traj_y = traj_data(:,2);

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

%% Velocities and trajectory

figure('Name', 'Velocities Map with Trajectory');

% Create and show smoothed velocities map
[X,Y] = meshgrid(1:columnas, 1:filas);
[Xq,Yq] = meshgrid(1:0.1:columnas, 1:0.1:filas);
velocities_smooth = interp2(X, Y, velocities_map, Xq, Yq, 'spline');
imagesc(velocities_smooth);
colormap(gray);
colorbar;
clim([0 1]);
axis equal;
hold on;

% Overlay trajectory
plot(traj_x*10, traj_y*10, 'r-', 'LineWidth', 2);
plot(traj_x(2:end-1)*10, traj_y(2:end-1)*10, 'r.', 'MarkerSize', 10);
plot(traj_x(1)*10, traj_y(1)*10, 'go', 'MarkerSize', 15, 'LineWidth', 2);
plot(traj_x(end)*10, traj_y(end)*10, 'ro', 'MarkerSize', 15, 'LineWidth', 2);

% Add labels and title
title('Velocities Map with Trajectory', 'FontSize', 12);
xlabel('X', 'FontSize', 11);
ylabel('Y', 'FontSize', 11);
set(gca, 'FontSize', 10);

% Add legend
legend('Trajectory', 'Waypoints', 'Start', 'End', 'Location', 'southeast');

hold off;


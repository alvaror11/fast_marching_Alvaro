clear;
clc;
close all;

main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Src\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";


cd(maps_folder)

mapa = readmatrix("MADRIDALTMAP.CSV");

figure;
surf(mapa,"EdgeColor","none")
fprintf('Reading terrain data from %s...\n', fullfile(maps_folder, "MADRIDALTMAP.CSV"));
mapa = readmatrix("MADRIDALTMAP.CSV");
fprintf('Terrain data read successfully.\n');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');

%% Ocupation maop txt

cd(files_folder)
fprintf('Reading occupation map...\n');
fid = fopen('occupation_map.txt', 'r');
if fid == -1
    error('Could not open occupation map file');
end

% Read dimensions from first line
dims = fscanf(fid, '%d', [1 3]);  % Read 3 numbers for 3D map
nx = dims(1);
ny = dims(2);
nz = dims(3);

% Initialize 3D matrix
occupation_map = zeros(nx, ny, nz);

% Skip "Layer 0:" text
fgetl(fid);
fgetl(fid);

% Read data layer by layer
for z = 1:nz
    % Read one layer
    for x = 1:nx
        line = fgetl(fid);
        values = sscanf(line, '%f');
        largo = length(values);
        if length(values) ~= dims(2)
            error(['Incorrect number of values in line ' num2str(x)]);
        end
        occupation_map(x,:,z) = values;
    end
    % Skip layer header except for last layer
    if z < nz
        fgetl(fid);  % Skip blank line
        header = fgetl(fid);  % Skip "Layer X:" text
    end
end
fclose(fid);

% Create figure for single occupation map slice
figure('Name', 'Occupation Map Z-Slice', 'Position', [100 100 600 500]);

% Choose which slice to show
z_slice = 10;  % Change this value to see different slices


slice = occupation_map(:,:,z_slice);
% Display the slice
imagesc(squeeze(occupation_map(:,:,z_slice)));
colormap([1 1 1; 0 0 0]);  % White for free space, black for obstacles
axis equal;
title(['Occupation Map - Z-Slice ' num2str(z_slice)]);
xlabel('X');
ylabel('Y');
grid on;
set(gca, 'GridColor', [0.8 0.8 0.8]);

% Optional: Add colorbar to show the scale
colorbar;

cd(main_folder)
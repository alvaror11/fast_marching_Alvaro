clear;
clc;
close all;

main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Src\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";


cd(maps_folder)

mapa = readmatrix("Mapa_50x50.CSV");

figure;
surf(mapa,"EdgeColor","none")
fprintf('Reading terrain data from %s...\n', fullfile(maps_folder, "MADRIDALTMAP.CSV"));

fprintf('Terrain data read successfully.\n');
axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'YDir', 'reverse');  
title('Terrain Map');

%% Ocupation map txt

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

%% Trajectory 3D
cd(files_folder)
fprintf('Reading trajectory data from %s...\n', fullfile(files_folder, 'trajectory3D.txt'));
fileID = fopen('trajectory3D.txt', 'r');
if fileID == -1
    error('Could not open trajectory file');
end

fprintf('Reading dimensions from first line...\n');
dims = fscanf(fileID, '%d');  % Read num_points and dimensions
num_points = dims(1);

fprintf('Initializing trajectory array...\n');
% Initialize trajectory array
trajectory = zeros(num_points, 3);

fprintf('Reading trajectory points...\n');
% Read trajectory points
for i = 1:num_points
    line = fgetl(fileID);
    if ischar(line)
        values = sscanf(line, '%f');
        if length(values) == 3
            trajectory(i,:) = values';
        end
    end
end
fclose(fileID);
fprintf('Trajectory data read successfully.\n');

%% Visual map+trajectory

% Create figure with terrain and trajectory
figure('Name', 'Madrid Terrain with 3D Trajectory', 'Position', [100 100 800 600]);

% Plot terrain with swapped X and Y coordinates
surf(mapa,"EdgeColor","none")

axis equal;
xlabel('X');
ylabel('Y');
zlabel('Z');
set(gca, 'YDir', 'reverse');  
hold on
% Plot trajectory (now coordinates will align correctly)
plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'r-', 'LineWidth', 2);

% Mark start and end points
plot3(trajectory(1,1), trajectory(1,2), trajectory(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(trajectory(end,1), trajectory(end,2), trajectory(end,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Adjust view
view(45, 30);
lighting gouraud;
camlight;
% Add legend
legend('Terrain', 'Trajectory', 'Start', 'End', 'Location', 'best');
hold off

cd(main_folder)
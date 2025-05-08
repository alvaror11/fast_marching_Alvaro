clear;
clc;
close all;

main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";

cd(maps_folder)

mapa = readmatrix("MADRIDALTMAP.CSV");

figure;
surf(mapa,"EdgeColor","none")fprintf('Reading terrain data from %s...\n', fullfile(maps_folder, "MADRIDALTMAP.CSV"));
mapa = readmatrix("MADRIDALTMAP.CSV");
fprintf('Terrain data read successfully.\n');

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

fprintf('Verifying trajectory data...\n');
% Verify trajectory data
if isempty(trajectory)
    error('No valid data found in trajectory file');
end
fprintf('Successfully read trajectory with %d points\n', size(trajectory, 1));
axis equal;

% Read trajectory data
cd(files_folder);
fileID = fopen('trajectory3D.txt', 'r');
if fileID == -1
    error('Could not open trajectory file');
end

% Read dimensions from first line
dims = fscanf(fileID, '%d');  % Read num_points and dimensions
num_points = dims(1);

% Initialize trajectory array
trajectory = zeros(num_points, 3);

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
cd(main_folder);

% Verify trajectory data
if isempty(trajectory)
    error('No valid data found in trajectory file');
end
fprintf('Successfully read trajectory with %d points\n', size(trajectory, 1));

% Create figure with terrain and trajectory
figure('Name', 'Madrid Terrain with 3D Trajectory', 'Position', [100 100 800 600]);

% Plot terrain
surf(mapa, 'EdgeColor', 'none', 'FaceAlpha', 0.7);  % Added transparency
hold on;

% Plot trajectory
plot3(trajectory(:,1) +1.5, trajectory(:,2) +1.5, trajectory(:,3) +1.5, 'r-', 'LineWidth', 2);

% Mark start and end points
plot3(trajectory(1,1), trajectory(1,2), trajectory(1,3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(trajectory(end,1), trajectory(end,2), trajectory(end,3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);

% Add labels and title
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Madrid Terrain with Flight Path');
colorbar;
axis equal;
grid on;

% Adjust view
view(45, 30);
lighting gouraud;
camlight;

% Add legend
legend('Terrain', 'Trajectory', 'Start', 'End', 'Location', 'best');



cd(main_folder)
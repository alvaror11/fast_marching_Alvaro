% Clear workspace and figures
clear;
clc;
%close all;

% Read the height map file
height_data = readmatrix('../../Archivos/height_map.txt');

% Create figure
figure('Name', 'Height Map Visualization', 'Position', [100 100 800 600]);

% Create color-coded plot
imagesc(height_data);
colormap(jet); % Use jet colormap for height visualization
colorbar; % Add colorbar to show height scale

% Add labels and title
xlabel('X coordinate');
ylabel('Y coordinate');
title('2D Height Map Visualization');

% Add grid
grid on;
axis equal;

% Add text showing min and max heights
min_height = min(height_data(:));
max_height = max(height_data(:));
text(10, size(height_data,1)+5, sprintf('Min height: %.1f, Max height: %.1f', min_height, max_height));


% Read the occupation map file
occupation_data = readmatrix('../../Archivos/occupation_map_2d.txt');

% Create figure
figure('Name', 'Occupation Map 2D Visualization', 'Position', [100 100 800 600]);

% Create binary visualization
imagesc(occupation_data);
colormap([0.8 0.9 1; 0.2 0.2 0.2]); % Light blue for free space, dark gray for obstacles
colorbar('Ticks', [0,1], 'TickLabels', {'Free', 'Obstacle'}); % Custom colorbar labels

% Add labels and title
xlabel('X coordinate');
ylabel('Y coordinate');
title('2D Occupation Map');

% Add grid and maintain proportions
grid on;
axis equal;
% Clear workspace and figures
clear;
clc;
close all;

% Read the height map file
height_data = readmatrix('../Archivos/height_map.txt');

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

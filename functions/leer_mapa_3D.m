clear;
clc;
close all;
main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
cd(files_folder)
% Load the .mat file containing the 3D map
load('mapa3d.mat');  % Adjust the filename as needed
% Assuming the variable in the .mat file is called 'mapa3D'
cd(main_folder);
% Get dimensions
[filas, columnas, altura] = size(Wgr);
Wgr = double(Wgr);

% Create 3D visualization
figure;

% Create meshgrid for visualization
[x, y, z] = meshgrid(1:columnas, 1:filas, 1:altura);

% Plot obstacles (1s) in red
p1 = patch(isosurface(x, y, z, Wgr, 0.5));
isonormals(x, y, z, Wgr, p1);
set(p1, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.3);

% Plot free space (0s) in blue
p2 = patch(isosurface(x, y, z, ~Wgr, 0.5));
isonormals(x, y, z, ~Wgr, p2);
set(p2, 'FaceColor', 'blue', 'EdgeColor', 'none', 'FaceAlpha', 0.1);

% Configure visualization
daspect([1 1 1]);
view(45, 30);
camlight;
lighting gouraud;
axis tight;
box on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Map Visualization');
legend([p1, p2], {'Obstacles', 'Free Space'}, 'Location', 'northeastoutside');
hold off;
% Save to text file
fileID = fopen('./mapa3D.txt', 'w');


% Write the map layer by layer
for k = 1:altura
    for i = 1:filas
        for j = 1:columnas
            fprintf(fileID, '%d ', Wgr(i,j,k));
        end
        fprintf(fileID, '\n');
    end
    % Add a separator between layers
    fprintf(fileID, '\n');
end

fclose(fileID);
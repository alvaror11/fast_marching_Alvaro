clear;
clc;
close all;

main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";

cd(maps_folder)

mapa = readmatrix("MADRIDALTMAP.CSV");

figure;
surf(mapa,"EdgeColor","none")
axis equal;

% Read the occupation matrix file
fid = fopen('mapa_MAD.txt','r');

% Read first line to get dimensions
first_line = fgetl(fid);
dims = sscanf(first_line, '%d %d %d');

% Initialize 3D matrix
matrix3D = zeros(dims(1), dims(2), dims(3));
current_layer = 0;
current_row = 1;

% Read the file line by line
while ~feof(fid)
    line = fgetl(fid);
    
    % Skip empty lines and layer headers
    if isempty(line) || all(isspace(line)) || contains(line, 'Layer')
        continue;
    end
    
    % Convert line to numbers
    numbers = str2num(line);
    
    if ~isempty(numbers)
        matrix3D(current_row,:,current_layer+1) = numbers;
        current_row = current_row + 1;
        
        % If we've completed a layer
        if current_row > dims(1)
            current_layer = current_layer + 1;
            current_row = 1;
        end
    end
end

fclose(fid);

% Create visualization
figure('Name', 'Occupation Matrix Visualization', 'Position', [100 100 1200 400]);



% Create visualization
figure('Name', 'Occupation Matrix Visualization', 'Position', [100 100 1200 400]);



% Plot 2: 3D view with height-based coloring
[x,y,z] = ind2sub(size(matrix3D), find(matrix3D));
scatter3(x, y, z, 20, z, 'filled', 'MarkerFaceAlpha', 0.6);  % Use z for color
colormap(gca, flipud(gray));  % Use inverted grayscale
colorbar;
title('3D View');
axis equal tight;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z (scaled)');
view(45,30);

% Improve visibility
set(gcf, 'Color', 'white');
set(gca, 'Color', [0.9 0.9 0.9]);  % Light gray background

cd(main_folder)
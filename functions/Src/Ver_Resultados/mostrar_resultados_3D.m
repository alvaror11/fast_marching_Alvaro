clear;
clc;
close all;
main_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Ver_Resultados";
files_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Archivos";
maps_folder = "C:\Users\alvar\OneDrive\Desktop\My code\repositorios\TFM_Code\fast_marching-master\functions\Mapas";

cd(files_folder)

%% Occupation map
%% Read and create 3D map from text file
cd(maps_folder);
fileID = fopen('mapa3D.txt', 'r');
if fileID == -1
    error('Could not open Mapa3D.txt file');
end

% Read dimensions from first line
ancho = 50;
largo = 50;
altura = 50;

% Initialize 3D matrix
Wgr = zeros(ancho, largo, altura);

% Read data layer by layer for original 50x50x50 map
for k = 1:altura  
    % Read each row of the layer
    for i = 1:ancho
        line = fgetl(fileID);
        if ~ischar(line)
            error(['Error reading line ' num2str(i) ' of layer ' num2str(k)]);
        end
        values = str2num(line);
        longitud = length(values);
        if length(values) ~= (largo)
            error(['Incorrect number of values in line ' num2str(i) ' of layer ' num2str(k)]);
        end
        Wgr(i,:,k) = values;
    end

    empty_line = fgetl(fileID); % Skip empty line between layers
end

% Read data layer by layer
% for k = 1:altura  
%     % Read one line containing the whole layer
%     line = fgetl(fileID);
%     if ~ischar(line)
%         error(['Error reading layer ' num2str(k)]);
%     end
% 
%     % Convert string to array of numbers
%     values = str2num(line);
%     if length(values) ~= (largo*ancho)
%         error(['Incorrect number of values in layer ' num2str(k) ...
%                '. Expected ' num2str(largo*ancho) ' but got ' num2str(length(values))]);
%     end
% 
%     % Reshape the values into a 2D layer and store in Wgr
%     layer_values = reshape(values, [largo, ancho])';
%     Wgr(:,:,k) = layer_values;
% 
%     % Skip empty line between layers if present
%     %empty_line = fgetl(fileID);
% end
fclose(fileID);
cd(main_folder);

%% Create 3D visualization
figure;
Wgr_viz = permute(Wgr, [2 1 3]);  % Swap X and Y dimensions
% Create meshgrid for visualization with correct orientation
[y, x, z] = meshgrid(1:largo, 1:ancho, 1:altura);

% Plot obstacles (1s) in red
p1 = patch(isosurface(y, x, z, Wgr_viz, 0.5));
isonormals(y, x, z, Wgr_viz, p1);
set(p1, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.9);

% Configure visualization
axis tight;
box on;
grid on;
xlabel('X');  % Y axis is horizontal (columns)
ylabel('Y');  % X axis is depth (rows)
zlabel('Z');  % Z axis is height (layers)
title('3D Map from Text File');

% Set view to match coordinate system
view(45, 30);
camlight;
lighting gouraud;
daspect([1 1 1]);
%% Velocities Map


%% Trayectoria

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
trajectory(1,1) = 10;

% Create trajectory visualization
figure;

Wgr_viz = permute(Wgr, [2 1 3]);  % Swap X and Y dimensions
% Create meshgrid for visualization with correct orientation
[y, x, z] = meshgrid(1:largo, 1:ancho, 1:altura);

% Plot obstacles (1s) in red
p1 = patch(isosurface(y, x, z, Wgr_viz, 0.5));
isonormals(y, x, z, Wgr_viz, p1);
set(p1, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
set(p1, 'FaceColor', 'red', 'EdgeColor', 'none', 'FaceAlpha', 0.9);
hold on;

% Plot trajectory with thick red line
h_traj = plot3(trajectory(:,1), trajectory(:,2), trajectory(:,3), 'b-', 'LineWidth', 3);

% Add start and end points with distinctive markers
h_start = plot3(trajectory(1,1), trajectory(1,2), trajectory(1,3), 'go', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'g');
h_end = plot3(trajectory(end,1), trajectory(end,2), trajectory(end,3), 'mo', ...
    'MarkerSize', 10, 'MarkerFaceColor', 'm');

% Configure visualization
axis tight;
box on;
grid on;
xlabel('X');
ylabel('Y');
zlabel('Z');
title('3D Trajectory with Obstacles');
legend([p1, h_traj, h_start, h_end], ...
       {'Obstacles', 'Trajectory', 'Start', 'End'}, ...
       'Location', 'northeastoutside');

% Set view angle and lighting
view(45, 30);
camlight;
lighting gouraud;
daspect([1 1 1]);

hold off;

cd(main_folder)

%% Times and Gradient Map Visualization
ancho = ancho + 2;
largo = largo + 2;
altura = altura +2;
% Read times map
cd(files_folder);
fileID = fopen('times_map3D.txt', 'r');
if fileID == -1
    error('Could not open times map file');
end

% Read dimensions
dims = fscanf(fileID, '%d', [1 3]);
times_map = zeros(ancho, largo, altura);

% Read dimensions
dims = fscanf(fileID, '%d', [1 3]);
times_map = zeros(ancho, largo, altura);

% Read data layer by layer
for k = 1:altura
    layer_header = fgetl(fileID); % Read "Layer n:" line
    if ~ischar(layer_header)
        error('Error reading layer header');
    end
    
    % Read each row of the layer
    for i = 1:ancho
        line = fgetl(fileID);
        if ~ischar(line)
            error(['Error reading line ' num2str(i) ' of layer ' num2str(k)]);
        end
        % Convert string to array of numbers
        values = str2num(line);  % Using str2num instead of sscanf
        if length(values) ~= largo
            error(['Incorrect number of values in line ' num2str(i) ' of layer ' num2str(k)]);
        end
        times_map(i,:,k) = values;
    end
    
    empty_line = fgetl(fileID); % Skip empty line between layers
end
fclose(fileID);
% Read gradient components
gradient_x = zeros(ancho, largo, altura);
gradient_y = zeros(ancho, largo, altura);
gradient_z = zeros(ancho, largo, altura);

% Read X component
fileID = fopen('gradient3D_x.txt', 'r');
if fileID == -1
    error('Could not open gradient_x file');
end
dims = fscanf(fileID, '%d', [1 3]); % Skip dimensions line
dims = fscanf(fileID, '%d', [1 3]); % Skip dimensions line
for k = 1:altura
    layer_header = fgetl(fileID); % Skip "Layer n:" line
    for i = 1:ancho
        line = fgetl(fileID);
        if ~ischar(line)
            error(['Error reading line ' num2str(i) ' of layer ' num2str(k)]);
        end
        values = str2num(line);
        largo = length(values);
        if length(values) ~= largo
            error(['Incorrect number of values in line ' num2str(i) ' of layer ' num2str(k)]);
        end
        gradient_x(i,:,k) = values;
    end
    empty_line = fgetl(fileID); % Skip empty line
end
fclose(fileID);

% Read Y component using same pattern
fileID = fopen('gradient3D_y.txt', 'r');
if fileID == -1
    error('Could not open gradient_y file');
end
dims = fscanf(fileID, '%d', [1 3]);
dims = fscanf(fileID, '%d', [1 3]);

for k = 1:altura
    layer_header = fgetl(fileID);
    for i = 1:ancho
        line = fgetl(fileID);
        if ~ischar(line)
            error(['Error reading line ' num2str(i) ' of layer ' num2str(k)]);
        end
        values = str2num(line);
        if length(values) ~= largo
            error(['Incorrect number of values in line ' num2str(i) ' of layer ' num2str(k)]);
        end
        gradient_y(i,:,k) = values;
    end
    empty_line = fgetl(fileID);
end
fclose(fileID);

% Read Z component using same pattern
fileID = fopen('gradient3D_z.txt', 'r');
if fileID == -1
    error('Could not open gradient_z file');
end
dims = fscanf(fileID, '%d', [1 3]);
dims = fscanf(fileID, '%d', [1 3]);

for k = 1:altura
    layer_header = fgetl(fileID);
    for i = 1:ancho
        line = fgetl(fileID);
        if ~ischar(line)
            error(['Error reading line ' num2str(i) ' of layer ' num2str(k)]);
        end
        values = str2num(line);
        if length(values) ~= largo
            error(['Incorrect number of values in line ' num2str(i) ' of layer ' num2str(k)]);
        end
        gradient_z(i,:,k) = values;
    end
    empty_line = fgetl(fileID);
end
fclose(fileID);

cd(main_folder);

% Example usage - change these values to visualize different slices

slice_dim = 'x';  % Options: 'x', 'y', or 'z'
slice_num = 20  ;   % Choose slice number within dimensions
slice_num = slice_num + 1;
% Visualize slice
visualizeSlice(times_map, gradient_x, gradient_y, gradient_z, slice_dim, slice_num, largo, ancho, altura);

min_time = 0;
max_time = 100;
visualizeTimesMap(times_map, slice_dim, slice_num, min_time, max_time);

cd(main_folder);

% Create slice visualization function
function visualizeSlice(data, grad_x, grad_y, grad_z, slice_dim, slice_num, largo, ancho, altura)
    figure;
    clipped_data = min(data, 400);
    
    switch slice_dim
        case 'x'
            slice_data = squeeze(clipped_data(:,slice_num,:));
            grad_data_1 = squeeze(grad_y(:,slice_num,:));  % Y component (horizontal)
            grad_data_2 = squeeze(grad_z(:,slice_num,:));  % Z component (vertical)
            
            % Display times map with origin at bottom left
            imagesc(slice_data);
            set(gca, 'YDir', 'normal');  % Z axis points up
            hold on
            
            % Create grid with Y horizontal and Z vertical
            [Y, Z] = meshgrid(1:largo, 1:altura);
            
            % Downsample for visualization
            skip = 1;
            Yd = Y(1:skip:end, 1:skip:end);
            Zd = Z(1:skip:end, 1:skip:end);
            Fy_d = grad_data_1(1:skip:end, 1:skip:end);
            Fz_d = grad_data_2(1:skip:end, 1:skip:end);
            
            % Plot vectors
            quiver(Yd, Zd, Fy_d, Fz_d, 2, 'w', 'LineWidth', 1.5);
            xlabel('Y'); ylabel('Z');
            
        case 'y'
            slice_data = squeeze(clipped_data(slice_num,:,:));
            grad_data_1 = -squeeze(grad_x(slice_num,:,:));  % X component (horizontal, negated for left direction)
            grad_data_2 = squeeze(grad_z(slice_num,:,:));   % Z component (vertical)
            
            % Display times map with origin at bottom right
            imagesc(fliplr(slice_data));  % Flip horizontally for right-to-left X
            set(gca, 'YDir', 'normal');   % Z axis points up
            hold on
            
            % Create grid with X horizontal (right to left) and Z vertical
            [X, Z] = meshgrid(1:largo, 1:altura);
            
            % Downsample for visualization
            skip = 1;
            Xd = X(1:skip:end, 1:skip:end);
            Zd = Z(1:skip:end, 1:skip:end);
            Fx_d = grad_data_1(1:skip:end, 1:skip:end);
            Fz_d = grad_data_2(1:skip:end, 1:skip:end);
            
            % Plot vectors with flipped X coordinates
            quiver(fliplr(Xd), Zd, fliplr(Fx_d), fliplr(Fz_d), 2, 'w', 'LineWidth', 1.5);
            xlabel('X'); ylabel('Z');
            
        case 'z'
            slice_data = clipped_data(:,:,slice_num);
            grad_data_1 = grad_x(:,:,slice_num);  % vertical component (X)
            grad_data_2 = grad_y(:,:,slice_num);  % horizontal component (Y)
            
            % Display times map
            imagesc(slice_data);
            hold on
            
            % Create grid matching image coordinates (Y horizontal, X vertical)
            [Y, X] = meshgrid(1:largo, 1:ancho);  % Y first for horizontal axis
            
            % Downsample for visualization
            skip = 1;
            Xd = X(1:skip:end, 1:skip:end);
            Yd = Y(1:skip:end, 1:skip:end);
            Fx_d = grad_data_1(1:skip:end, 1:skip:end);  % X gradient (vertical)
            Fy_d = grad_data_2(1:skip:end, 1:skip:end);  % Y gradient (horizontal)
            
            % Plot vectors with correct components
            quiver(Yd, Xd, Fy_d, Fx_d, 2, 'w', 'LineWidth', 1.5);
            
            xlabel('Y'); ylabel('X');  % Label axes appropriately

    end
    
    % Configure visualization
    title(['Times Map - ' upper(slice_dim) ' Slice ' num2str(slice_num)]);
    colormap(jet);
    c = colorbar;
    ylabel(c, 'Time');
    caxis([0 400]);
    view(2);
    axis equal tight;
    grid on;
end


function visualizeTimesMap(data, slice_dim, slice_num, min_time, max_time)
    figure;
    
    switch slice_dim
        case 'x'
            slice_data = squeeze(data(:,slice_num,:))';
            xlabel('Y'); ylabel('Z');
        case 'y'
            slice_data = squeeze(data(slice_num,:,:))';
            xlabel('X'); ylabel('Z');
        case 'z'
            slice_data = data(:,:,slice_num);
            xlabel('X'); ylabel('Y');
    end
    
    % Create masked and modified data for visualization
    modified_data = slice_data;
    obstacle_mask = slice_data > max_time;
    valid_mask = ~obstacle_mask & slice_data >= min_time;
    
    % Create custom colormap for better visualization of small differences
    n_colors = 256;
    custom_map = [
        % First 70% of colormap for values 0-50
        flipud(copper(round(0.7*n_colors))); 
        % Remaining 30% for values 50-400
        flipud(hot(round(0.3*n_colors)))
    ];
    
    % Add gray color for obstacles
    custom_map = [custom_map; 0.5 0.5 0.5];
    
    % Normalize valid data to enhance small differences
    modified_data(valid_mask) = (slice_data(valid_mask) - min_time) / (max_time - min_time);
    modified_data(obstacle_mask) = 2;  % Value beyond colormap range for obstacles
    
    % Create visualization
    imagesc(modified_data);
    colormap(custom_map);
    
    % Configure colorbar with custom ticks
    c = colorbar;
    ylabel(c, 'Time (s)');
    
    % Calculate colorbar ticks for better readability
    tick_positions = linspace(0, 1, 10);
    tick_values = linspace(min_time, max_time, 10);
    set(c, 'Ticks', [tick_positions 2], ...
           'TickLabels', [arrayfun(@(x) sprintf('%.1f', x), tick_values, 'UniformOutput', false) {'Obs'}]);
    
    title(['Times Map - ' upper(slice_dim) ' Slice ' num2str(slice_num)]);
    axis equal tight;
    grid on;
    %set(gca, 'YDir', 'normal');
    
    % Add numerical values if map is not too large
    [rows, cols] = size(slice_data);
    if max(rows, cols) < 20
        for i = 1:rows
            for j = 1:cols
                if ~obstacle_mask(i,j)
                    text(j, i, num2str(slice_data(i,j),'%.1f'), ...
                        'HorizontalAlignment', 'center', ...
                        'Color', 'k', ...
                        'FontWeight', 'bold');
                else
                    text(j, i, 'X', ...
                        'HorizontalAlignment', 'center', ...
                        'Color', 'w', ...
                        'FontWeight', 'bold');
                end
            end
        end
    end
end
